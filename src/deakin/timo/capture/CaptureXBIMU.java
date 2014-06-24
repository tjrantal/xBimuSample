/*
	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

	N.B.  the above text was copied from http://www.gnu.org/licenses/gpl.html
	unmodified. I have not attached a copy of the GNU license to the source...

    Copyright (C) 2011-2013 Timo Rantalainen
*/

package deakin.timo.capture;

import deakin.timo.xBimuSample.XBimuSample; /*Import ui*/
import jssc.SerialPort;
import jssc.SerialPortException;
import java.util.ArrayList;
import java.util.Arrays;
import java.io.BufferedOutputStream;
import deakin.timo.DrawImage.*;
import deakin.timo.visualizeAxes.*;			/*Visualize axes jogl*/
import deakin.timo.visualizeAxes.utils.*;	/*Quaternion*/
import deakin.timo.madgwickAHRS.*;							/*Madgwick AHRS*/
import java.text.DecimalFormat;	//For rounding text

public class CaptureXBIMU extends Capture{
	int sensorByteLength = 21;	/**sensor buffer length*/
	int batteryByteLength = 5;	/**battery byte length, to be ignored for now*/
	int maxByteLength = 21;	/**Max buffer length*/
	int minByteLength = 5;	/**Min buffer length*/
	byte[] dataInBuffer = null;		/**Read data into temp buffer*/
	int bufferPointer = 0;
	short[] valuesIn;
	short[] batteryIn;
	int channelsToVisualize = 6;
	DrawImage drawImage3;
	DrawImage drawImage4;
	VisualizeAxes orientationWindow;
	private Quaternion quat = null;
	double[] Z = {0d,0d,1d};
	Quaternion zAxis = null;
	private MadgwickAHRS madgwickAHRS = null;
	private double sFreq = 256;		/**Set this with the x-BIMU software*/
	DecimalFormat dfo;

	/*Constructor*/
	public CaptureXBIMU(XBimuSample mainProgram,SerialPort serialPort,BufferedOutputStream oStream, DrawImage drawImage1, DrawImage drawImage2,DrawImage drawImage3,DrawImage drawImage4,VisualizeAxes orientationWindow){
		super(mainProgram,serialPort,oStream,drawImage1,drawImage2);
		this.drawImage3 = drawImage3;
		this.drawImage4 = drawImage4;
		this.orientationWindow = orientationWindow;
		noChannels = 9;
		setChannels(noChannels);
		valuesIn = new short[noChannels];
		batteryIn = new short[1];
		zAxis = new Quaternion(0,Z[0],Z[1],Z[2]);
		madgwickAHRS = new MadgwickAHRSIMU(0.1d, new double[]{1,0,0,0}, sFreq);
		dfo = new DecimalFormat("0.0");
	}



	@Override
	public void run() {
		try{

			while (keepSampling) {
				/*Read at least maxByteLength bytes from the serial port*/
				int bytesReady = serialPort.getInputBufferBytesCount();
				while (keepSampling && bytesReady < maxByteLength){
					try{
						Thread.sleep(1);	/*Sleep for 1 ms...*/
						bytesReady = serialPort.getInputBufferBytesCount();
					} catch (Exception err){
						System.out.println("Sleeping and checking buffer byte count failed");
						break;
					}
				}
				byte[] tempBuffer = serialPort.readBytes(bytesReady);

				if (dataInBuffer == null){
					//System.out.println("dataInBuffer null");
					dataInBuffer = copyByteArray(tempBuffer);
				}else{
					//System.out.println("Existing data");
					byte[] tb = copyByteArray(dataInBuffer);
					dataInBuffer = new byte[tb.length+tempBuffer.length];
					for (int i = 0;i<tb.length;++i){
						dataInBuffer[i] = tb[i];
					}
					for (int i = 0;i<tempBuffer.length;++i){
						dataInBuffer[tb.length+i] = tempBuffer[i];
					}
					//System.out.println("Appended "+Integer.toString(dataInBuffer.length));
				}

				//System.out.println("decoding "+Integer.toString(dataInBuffer.length));

				/*Handle the byte buffer read*/
				bufferPointer = 0;
				//System.out.println("Start going through the buffer");
				if (dataInBuffer != null){
					Boolean goOn = true;
					while (goOn && dataInBuffer.length-bufferPointer > minByteLength){

						/**Search for the first byte with correct char*/
						while (bufferPointer < (dataInBuffer.length-minByteLength) && dataInBuffer[bufferPointer] != 'S' && dataInBuffer[bufferPointer] != 'B'){
							++bufferPointer;
						}
						//System.out.println("Going to switch");
						/**Select how to handle data*/
						if (bufferPointer < (dataInBuffer.length-minByteLength)){
							switch (dataInBuffer[bufferPointer]){
								case 'B':
									//System.out.println("B chosen");
									if(bufferPointer < (dataInBuffer.length-batteryByteLength)){
										//System.out.println("Case B");
										byte[] batteryPacket = new byte[batteryByteLength];
										int tempInd = 0;
										for (int i = bufferPointer;i<bufferPointer+batteryByteLength;++i){
											batteryPacket[tempInd] = dataInBuffer[i];
											++tempInd;
										}
										if (checkByteSumValue(batteryPacket) == 0){
											decodeBatteryPacket(batteryPacket);	/**Decode battery packet*/
											bufferPointer+=batteryByteLength;	/*Set the pointer to the next packet*/
											//System.out.println("BatteryPacket "+Byte.toString(batteryPacket[batteryPacket.length-2]));
										}else{
											++bufferPointer;	/**False alarm*/
											System.out.println("BatteryPacket checksumFail"+Byte.toString(checkByteSumValue(batteryPacket)));
										}

									}else{
										goOn = false;
										//System.out.println("B chosen too little data "+Integer.toString(dataInBuffer.length-bufferPointer));
									}
									break;
								case 'S':

									if(bufferPointer < (dataInBuffer.length-sensorByteLength)){
										//System.out.println("S chosen "+Integer.toString(dataInBuffer.length-bufferPointer));
										byte[] sensorPacket = new byte[sensorByteLength];
										int tempInd = 0;
										for (int i = bufferPointer;i<bufferPointer+sensorByteLength;++i){
											sensorPacket[tempInd] = dataInBuffer[i];
											++tempInd;
										}
										if (checkByteSumValue(sensorPacket) == 0){
											decodeSensorPacket(sensorPacket);	/**Decode sensor packet*/
											bufferPointer+=sensorByteLength;	/*Set the pointer to the next packet*/
										}else{
											++bufferPointer;	/**False alarm*/
											System.out.println("SensorPacket checksumFail "+Byte.toString(checkByteSumValue(sensorPacket)));
										}
									}else{
										goOn = false;
										//System.out.println("S chosen too little data "+Integer.toString(dataInBuffer.length-bufferPointer));
									}

									break;
							}
						}else{
							goOn = false;
						}
					}
					//System.out.println("Save remaining bytes "+Integer.toString(dataInBuffer.length-bufferPointer));
					if (bufferPointer < (dataInBuffer.length)){
						byte[] tempBuffer2 = new byte[dataInBuffer.length-bufferPointer];
						int inde = 0;
						for (int i = bufferPointer;i<dataInBuffer.length;++i){
							tempBuffer2[inde] = dataInBuffer[i];
							++inde;
						}
						dataInBuffer = tempBuffer2;
					}else{
						dataInBuffer = null;
					}

				}else{
					System.out.println("Read null");
				}

			}
			/*All done*/
			doneSampling();
		}catch (SerialPortException ex){
			System.out.println("Capture run error");
			System.out.println(ex);
		}
	}

	private byte[] copyByteArray(byte[] arrayIn){
		byte[] copyOfArrayIn = new byte[arrayIn.length];
		for (int i = 0; i<arrayIn.length;++i){
			copyOfArrayIn[i] = arrayIn[i];
		}
		//System.out.println("Copied an array "+Integer.toString(copyOfArrayIn.length));
		return copyOfArrayIn;
	}

	/**Decode battery packet*/
	private void decodeBatteryPacket(byte[] bufferIn){
		//Battery mV
		batteryIn[0] =(short)((((short) (bufferIn[1] & 0xff))<< 8) | (bufferIn[2] & 0xff));
		byte currentCount = bufferIn[3];
		//update voltage window
		drawImage4.plotNumber(((float) batteryIn[0])/1000f);
	}

	/**Decode sensor packet*/
	private void  decodeSensorPacket(byte[] bufferIn){
		/*Check sampling machine time*/
		currentNanos = System.nanoTime();
		for (int i = 0;i<noChannels;++i){
			valuesIn[i] =(short)((((short) (bufferIn[2*i+1] & 0xff))<< 8) | (bufferIn[2*i+2] & 0xff));
		}
		byte currentCount = bufferIn[2*noChannels+1];

		/*ORIENTATION*/
		calculateQuaternion();

		/*Assign the values to ArrayLists*/
		for (int i = 0; i < noChannels;++i){
			if (capturedDataPoints < historyLength){
				data.get(i).set(capturedDataPoints,(int) valuesIn[i]);
			}else{
				data.get(i).add((int) valuesIn[i]);
				data.get(i).remove(0);	//Remove the oldest value
			}
		}

		if (capturedDataPoints < historyLength){
			++capturedDataPoints;
		}

		/*print results to file*/
		if (oStream != null && mainProgram.continueSampling){
			try{
				/*Change the int[] to byte[] for writing*/
				byte[] valueBytes = new byte[(valuesIn.length+1)*2];	//Include battery voltage
				for (int i = 0; i<valuesIn.length;++i){
					for (int j = 0; j<2;++j){
						valueBytes[i*2+j] =(byte) (0xff & (valuesIn[i]>>(8*j)));	//Change from java Big endian to little endian
					}
				}
				//Battery voltage
				for (int j = 0; j<2;++j){
					valueBytes[valuesIn.length*2+j] =(byte) (0xff & (batteryIn[0]>>(8*j)));	//Change from java Big endian to little endian
				}
				//System.out.println("BAttery "+Short.toString(batteryIn[0]));
				//Write the channel results
				oStream.write(valueBytes);
				/**write sampling nanos, score, timerStarted, dualTaskEnabled*/
				byte[] extra = new byte[8+1+1+1+2]; //long,  byte, byte,byte,short
				extra = longToBytes(currentNanos,extra,0);
				extra[8] = mainProgram.timerStarted ? (byte) 1:(byte) 0;
				extra[9] =  mainProgram.dualTaskEnabled ? (byte) 1:(byte) 0;
				extra[10] = currentCount;
				//current task
				for (int j = 0; j<2;++j){
					extra[11+j] =(byte) (0xff & (mainProgram.currentTask>>(8*j)));	//Change from java Big endian to little endian
				}
				//Write the extra id info
				oStream.write(extra);
				oStream.flush();

			} catch (Exception err){
				//System.out.println("Couldn't write sensor data");
			}
		}

		++plotCount;
		if (plotCount >= plotEveryNth){
			plotCount -= plotEveryNth;
			//mainProgram.textLabel.setText(Long.toString(timeStamp));
			drawImage1.clearPlot();
			drawImage2.clearPlot();
			drawImage3.clearPlot();
			/*Plot the traces*/
			for (int i = 0; i < 3;++i){
				drawImage1.plotTrace(data.get(i).toArray(new Integer[]{}),i);
			}

			for (int i = 3; i < 6;++i){
				drawImage2.plotTrace(data.get(i).toArray(new Integer[]{}),i-3);
			}
			String[] tempVals = new String[3];
			for (int i = 6; i< 9; ++i){
			   drawImage3.plotTrace(data.get(i).toArray(new Integer[]{}),i-6);
			   tempVals[i-6] = dfo.format((data.get(i)).get((data.get(i)).size()-1));
			}
			drawImage1.plotString(new String("Gyro"));
			drawImage2.plotString(new String("Accelerometer"));
			drawImage3.plotString(new String("Magnetometer"));
			drawImage3.plotString(tempVals[0],0.7,0.1,0);
			drawImage3.plotString(tempVals[1],0.7,0.2,1);
			drawImage3.plotString(tempVals[2],0.7,0.3,2);
			drawImage1.paintImageToDraw();
			drawImage2.paintImageToDraw();
			drawImage3.paintImageToDraw();
			//System.out.print("Last value "+value+" timeStamp "+timeStamp+"\r");
		}

	}

	private void calculateQuaternion(){
		/*Calculate and set quaternion!!*/
		//constants.channelOrder = [4,5,6,1,2,3];
		//constants.scalings = [1/1000 1/1000 1/1000 1/(10*180)*pi 1/(10*180)*pi 1/(10*180)*pi];
		/*Scale the values*/
		double[] imuData = new double[6];
		imuData[0] = ((double)valuesIn[3])/1000d;
		imuData[1] = ((double)valuesIn[4])/1000d;
		imuData[2] = ((double)valuesIn[5])/1000d;
		imuData[3] = ((double)valuesIn[0])/(10d*180d)*Math.PI;
		imuData[4] = ((double)valuesIn[1])/(10d*180d)*Math.PI;
		imuData[5] = ((double)valuesIn[2])/(10d*180d)*Math.PI;

		double initTheta;
		double[] rotAxis;
		/*The initial round*/
		if (quat == null){
			//Set the initial orientation according to first sample of accelerometry

			initTheta =Math.acos(dot(normalize(new double[]{imuData[0],imuData[1],imuData[2]}),Z));
			rotAxis = cross(normalize(new double[]{imuData[0],imuData[1],imuData[2]}),Z);
			//System.out.println("X "+Double.toString(rotAxis[0]) +" Y "+Double.toString(rotAxis[1])+" Z "+Double.toString(rotAxis[2])+" norm "+Double.toString(norm(rotAxis))+" cos "+Double.toString(Math.cos(initTheta/2d))+" "+Double.toString(initTheta));
			if (norm(rotAxis) != 0){
				rotAxis = normalize(rotAxis);
				quat = new Quaternion(Math.cos(initTheta/2d),Math.sin(initTheta/2d)*rotAxis[0],Math.sin(initTheta/2d)*rotAxis[1],Math.sin(initTheta/2d)*rotAxis[2]);
			}else{
				quat = new Quaternion(1d,0d,0d,0d);
				quat = quat.conjugate();		/**The conjugate is used in visualization*/
			}
			madgwickAHRS.setOrientationQuaternion(quat.getDouble());

		}else{
			/**Update the orientation with madgwick gradient descent algorithm*/
			madgwickAHRS.AHRSUpdate(new double[] {imuData[3],imuData[4],imuData[5],imuData[0],imuData[1],imuData[2]});
			double[] tempQ = madgwickAHRS.getOrientationQuaternion();
			quat = new Quaternion(tempQ[0],-tempQ[1],-tempQ[2],-tempQ[3]);	/**Return the conjugate, because that's what's used in the visualization*/
		}

		if (quat != null){
			orientationWindow.setRotationQuaternion(quat.getFloat());
		}
	}


	/**Quaternion orientation calculations stuff*/
	private double[] normalize(double[] a){
		double magnitude = norm(a);
		for (int i = 0;i<a.length;++i){
			a[i]= (a[i]/magnitude);
		}
		return a;
	}

	private double[] diff(double[] arrIn){
		double[] arrOut = new double[arrIn.length-1];
		for (int i = 0;i<arrIn.length-1;++i){
			arrOut[i] = arrIn[i+1]-arrIn[i];
		}
		return arrOut;
	}

	private double mean(double[] a){
		double b = 0;
		for (int i = 0;i<a.length;++i){
			b += a[i]/((double)a.length);
		}
		return b;
	}

	private double dot(double[] a, double[] b){
		return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
	}

	private double[] cross(double[] a, double[] b){
		double[] c =new double[3];
		c[0] = (a[1]*b[2]-a[2]*b[1]);
		c[1] = (a[2]*b[0]-a[0]*b[2]);
		c[2] = (a[0]*b[1]-a[1]*b[0]);
		return c;
	}

	private double norm(double[] a){
		double b=0;
		for (int i = 0;i<a.length;++i){
			b+=a[i]*a[i];
		}
		return Math.sqrt(b);
	}

	private double norm(double a, double b, double c){
		return norm(new double[]{a,b,c});
	}

	@Override
	protected void doneSampling(){
		//mainProgram.tare.setEnabled(false);
		/*Close the save file*/
		try{
			serialPort.purgePort(SerialPort.PURGE_RXCLEAR);		/*Try to eliminate any extraneous data*/
			if (oStream != null){
				oStream.flush();
				oStream.close();
				oStream = null;
			}
		}catch (Exception ex){
			System.out.println(ex);
		}
	}



	/*
	CheckSum from x-io
	        private byte CalcChecksum(byte packetLength)
        {
            byte tempRxBufIndex = (byte)(binBufIndex - packetLength);
            byte checksum = 0;
            while (tempRxBufIndex != binBufIndex)
            {
                checksum ^= binBuf[tempRxBufIndex++];
            }
            return checksum;
        }

	*/

	public byte checkByteSumValue(byte[] bufferIn){
		byte checksum = 0;
		byte tempRxBufIndex = 0;
		while (tempRxBufIndex < bufferIn.length){
			checksum ^= bufferIn[tempRxBufIndex];
			++tempRxBufIndex;
        }
		return checksum;
	}



}

