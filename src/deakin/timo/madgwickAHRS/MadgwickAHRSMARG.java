/**
	Port of Seb Madqwick's open source IMU AHRS C implementation http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
	Decided to changed to double precision instead of float. Don't know whether it matters.
	Written by Timo Rantalainen 2014 tjrantal at gmail dot com
	Licensed with GPL 3.0 (https://www.gnu.org/copyleft/gpl.html)
*/
package deakin.timo.madgwickAHRS;
public class MadgwickAHRSMARG extends MadgwickAHRS{
	/**
		Constructor
		@param beta algorithm gain
		@param q orientation quaternion
		@param samplingFreq Sampling frequency
	*/
	public MadgwickAHRSMARG(double beta, double[] q, double samplingFreq){
		super(beta,q,samplingFreq);
	}

	/**
		Update the orientation according to the latest set of MARG measurements
		@param AHRSdata The latest set of MARG data [0-2] gyro, [3-5] accelerometer, [6-8] magnetometer
	*/
	@Override
	public void  AHRSUpdate(double[] AHRSdata){
		double recipNorm;
		double[] s		= new double[4];
		double[] qDot	= new double[4];
		double[] h		= new double[2];
		double[] _2qm	= new double[4];
		double[] _2b	= new double[2];
		double[] _4b	= new double[2];
		double[] _2q	= new double[6];
		double[] qq		= new double[10];

		// Rate of change of quaternion from gyroscope
		qDot[0] = 0.5d * (-q[1] * AHRSdata[0] - q[2] * AHRSdata[1] - q[3] * AHRSdata[2]);
		qDot[1] = 0.5d * (q[0] * AHRSdata[0] + q[2] * AHRSdata[2] - q[3] * AHRSdata[1]);
		qDot[2] = 0.5d * (q[0] * AHRSdata[1] - q[1] * AHRSdata[2] + q[3] * AHRSdata[0]);
		qDot[3] = 0.5d * (q[0] * AHRSdata[2] + q[1] * AHRSdata[1] - q[2] * AHRSdata[0]);

		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((AHRSdata[3] == 0d) && (AHRSdata[4] == 0d) && (AHRSdata[5] == 0d))) {

			// Normalise accelerometer measurement
			recipNorm	= invSqrt(AHRSdata[3] * AHRSdata[3] + AHRSdata[4] * AHRSdata[4] + AHRSdata[5] * AHRSdata[5]);
			AHRSdata[3]	*= recipNorm;
			AHRSdata[4]	*= recipNorm;
			AHRSdata[5]	*= recipNorm;

			//Normalize magnetometer measurement
			recipNorm	= invSqrt(AHRSdata[6] * AHRSdata[6] + AHRSdata[7] * AHRSdata[7] + AHRSdata[8] * AHRSdata[8]);
			AHRSdata[6]	*= recipNorm;
			AHRSdata[7]	*= recipNorm;
			AHRSdata[8]	*= recipNorm;

			// Auxiliary variables to avoid repeated arithmetic
			_2qm[0] = 2d * q[0]*AHRSdata[6];
			_2qm[1] = 2d * q[0]*AHRSdata[7];
			_2qm[2] = 2d * q[0]*AHRSdata[8];
			_2qm[3] = 2d * q[1]*AHRSdata[6];

			_2q[0] = 2d * q[0];
			_2q[1] = 2d * q[1];
			_2q[2] = 2d * q[2];
			_2q[3] = 2d * q[3];
			_2q[4] = 2d * q[0]*q[2];
			_2q[5] = 2d * q[2]*q[3];

			qq[0] = q[0] * q[0];
			qq[1] = q[0] * q[1];
			qq[2] = q[0] * q[2];
			qq[3] = q[0] * q[3];
			qq[4] = q[1] * q[1];
			qq[5] = q[1] * q[2];
			qq[6] = q[1] * q[3];
			qq[7] = q[2] * q[2];
			qq[8] = q[2] * q[3];
			qq[9] = q[3] * q[3];

			// Reference direction of Earth's magnetic field
			// Gradient decent algorithm corrective step
			s[0] = -_2q[2] * (2.0f * qq[6] - _2q[4] - AHRSdata[3]) + _2q[1] * (2.0f * qq[1] + _2q[5] - AHRSdata[4]) - _2b[1] * q[2] * (_2b[0] * (0.5f - qq[7] - qq[9]) + _2b[1] * (qq[6] - qq[2]) - AHRSdata[6]) + (-_2b[0] * q[3] + _2b[1] * q[1]) * (_2b[0] * (qq[5] - qq[3]) + _2b[1] * (qq[1] + qq[8]) - AHRSdata[7]) + _2b[0] * q[2] * (_2b[0] * (qq[2] + qq[6]) + _2b[1] * (0.5f - qq[4] - qq[7]) - AHRSdata[8]);
			s[1] = _2q[3] * (2.0f * qq[6] - _2q[4] - AHRSdata[3]) + _2q[0] * (2.0f * qq[1] + _2q[5] - AHRSdata[4]) - 4.0f * q[1] * (1 - 2.0f * qq[4] - 2.0f * qq[7] - AHRSdata[5]) + _2b[1] * q[3] * (_2b[0] * (0.5f - qq[7] - qq[9]) + _2b[1] * (qq[6] - qq[2]) - AHRSdata[6]) + (_2b[0] * q[2] + _2b[1] * q[0]) * (_2b[0] * (qq[5] - qq[3]) + _2b[1] * (qq[1] + qq[8]) - AHRSdata[7]) + (_2b[0] * q[3] - _4b[1] * q[1]) * (_2b[0] * (qq[2] + qq[6]) + _2b[1] * (0.5f - qq[4] - qq[7]) - AHRSdata[8]);
			s[2] = -_2q[0] * (2.0f * qq[6] - _2q[4] - AHRSdata[3]) + _2q[3] * (2.0f * qq[1] + _2q[5] - AHRSdata[4]) - 4.0f * q[2] * (1 - 2.0f * qq[4] - 2.0f * qq[7] - AHRSdata[5]) + (-_4b[0] * q[2] - _2b[1] * q[0]) * (_2b[0] * (0.5f - qq[7] - qq[9]) + _2b[1] * (qq[6] - qq[2]) - AHRSdata[6]) + (_2b[0] * q[1] + _2b[1] * q[3]) * (_2b[0] * (qq[5] - qq[3]) + _2b[1] * (qq[1] + qq[8]) - AHRSdata[7]) + (_2b[0] * q[0] - _4b[1] * q[2]) * (_2b[0] * (qq[2] + qq[6]) + _2b[1] * (0.5f - qq[4] - qq[7]) - AHRSdata[8]);
			s[3] = _2q[1] * (2.0f * qq[6] - _2q[4] - AHRSdata[3]) + _2q[2] * (2.0f * qq[1] + _2q[5] - AHRSdata[4]) + (-_4b[0] * q[3] + _2b[1] * q[1]) * (_2b[0] * (0.5f - qq[7] - qq[9]) + _2b[1] * (qq[6] - qq[2]) - AHRSdata[6]) + (-_2b[0] * q[0] + _2b[1] * q[2]) * (_2b[0] * (qq[5] - qq[3]) + _2b[1] * (qq[1] + qq[8]) - AHRSdata[7]) + _2b[0] * q[1] * (_2b[0] * (qq[2] + qq[6]) + _2b[1] * (0.5f - qq[4] - qq[7]) - AHRSdata[8]);

			//Insert corrective step...

			recipNorm = invSqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2] + s[3] * s[3]); // normalise step magnitude
			s[0] *= recipNorm;
			s[1] *= recipNorm;
			s[2] *= recipNorm;
			s[3] *= recipNorm;

			// Apply feedback step
			qDot[0] -= beta * s[0];
			qDot[1] -= beta * s[1];
			qDot[2] -= beta * s[2];
			qDot[3] -= beta * s[3];
		}

		// Integrate rate of change of quaternion to yield quaternion
		q[0] += qDot[0] * (1.0f / samplingFreq);
		q[1] += qDot[1] * (1.0f / samplingFreq);
		q[2] += qDot[2] * (1.0f / samplingFreq);
		q[3] += qDot[3] * (1.0f / samplingFreq);

		// Normalise quaternion
		recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
		q[0] *= recipNorm;
		q[1] *= recipNorm;
		q[2] *= recipNorm;
		q[3] *= recipNorm;
	}
}