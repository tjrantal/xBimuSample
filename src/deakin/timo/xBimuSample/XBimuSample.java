/*
	Use simple serial connector library to access bluetooth serial port http://code.google.com/p/java-simple-serial-connector/
	Use javax.swing for UI creation
*/
package deakin.timo.xBimuSample;
/*Swing classes*/
import javax.swing.*;		//GUI commands swing
import java.awt.event.*; 	//Events & Actionlistener
import java.io.*;				//File IO
import java.lang.Math;
import java.awt.*;
import java.awt.geom.Line2D;
import javax.swing.event.*;
import javax.swing.border.*;
/*Simple serial connector classes*/
import jssc.SerialPort;
import jssc.SerialPortException;
import jssc.SerialPortList;
/*Capture thread*/
import deakin.timo.capture.*;
/*DrawImage*/
import deakin.timo.DrawImage.*;
/*Choosing and saving a file*/
import javax.swing.SwingUtilities;
import javax.swing.filechooser.*;
import java.util.prefs.Preferences;		/*Saving the file save path -> no need to rebrowse...*/
/*Timestamp*/
import java.sql.Timestamp;
import java.util.Date;

/*3D visualization*/
import javax.media.opengl.*;			/*jogl Capabilities*/
import deakin.timo.visualizeAxes.*;			/*Visualize axes jogl*/

/*
	ActionListener for actions with Swing buttons etc.
	KeyListener to get key events for starting and stopping sampling
	WindowListener to close the save file gracefully at the end.

*/

public class XBimuSample extends JPanel implements ActionListener, WindowListener{
	private JComboBox[] comPortDropDownMenu;
	public JButton chooseSaveFile;			/*For saving the results to a file*/
	private JFileChooser fileChooser;		/*Selecting the file to save to*/
	private Preferences preferences;		/**Saving the default file path*/
	private final String keySP = "SP";
	private String savePath;
	private JButton connectBluetooth[];
	//public JButton tare;
	public JButton beginSampling;
	public JButton endSampling;

	public Boolean dualTaskEnabled = false;	/**Used to enable dual task timer*/
	public Boolean timerStarted = false; /**Used to indicate whether dual task is in session*/
	private JButton closeBluetooth[];

	private JComboBox taskDropDown;
	public short currentTask = 0;

	private String[] portToConnectTo;
	public DrawImage[] drawImages;
	public VisualizeAxes[] orientationWindow;
	public Boolean continueSampling = false;
	public SerialPort[] serialPort;
	public JLabel textLabel;
	private static int imWidth = 800;
	private static int imHeight = 400;
	public BufferedOutputStream[] oStream;
	/**For results; windows*/

	public long startTime;

	private Capture[] capture;
	private Thread[] captureThread;
	//private Date date;
	private File file = null;	/*Used to get a file name for fileoutputstream*/

	/*3D visualization*/
	public VisualizeAxes visualizeWindow;

	private JTextArea idArea;

	public XBimuSample(JFrame parentFrame){
		init();	/*Get default save file folder, and init file chooser*/

		JPanel[] buttons = new JPanel[2];
		for (int i = 0; i< buttons.length;++i){
			buttons[i] = new JPanel();
		}

		/*buttons[0] ID: idArea saveFolder beginSampling endSampling*/

		/*ID textArea*/
		buttons[0].add(new JLabel("ID:"));
		idArea = new JTextArea("Subject ID",1,20);
		buttons[0].add(idArea);

		/**Add save file chooser*/
		chooseSaveFile = new JButton("Select save file");
		chooseSaveFile.setMnemonic(KeyEvent.VK_S);
		chooseSaveFile.setActionCommand("chooseSaveFile");
		chooseSaveFile.addActionListener(this);
		chooseSaveFile.setToolTipText("Press to select the file to save results to");
		chooseSaveFile.setEnabled(true);
		buttons[0].add(chooseSaveFile);

		//date = new java.util.Date();	/*Date object to get a timestamp*/

		/*Begin button*/
		beginSampling= new JButton("Begin sampling");
		beginSampling.setMnemonic(KeyEvent.VK_B);
		beginSampling.setActionCommand("beginSampling");
		beginSampling.addActionListener(this);
		beginSampling.setToolTipText("Press to begin sampling");
		beginSampling.setEnabled(false);
		buttons[0].add(beginSampling);

		/*End button*/
		endSampling= new JButton("End sampling");
		endSampling.setMnemonic(KeyEvent.VK_E);
		endSampling.setActionCommand("endSampling");
		endSampling.addActionListener(this);
		endSampling.setToolTipText("Press to end sampling");
		endSampling.setEnabled(false);
		buttons[0].add(endSampling);

		/*buttons[0] ID: idArea beginSampling endSampling*/

		/*Add dropbox list of com ports*/
		comPortDropDownMenu = new JComboBox[1];
		connectBluetooth	= new JButton[1];
		closeBluetooth		= new JButton[1];
		String[] comPorts = SerialPortList.getPortNames();	//List com ports
		String[] portLabel = new String[]{"xBIMU com port selection"};
		for (int c = 0; c<comPortDropDownMenu.length;++c){
			buttons[c+1].add(new JLabel(portLabel[c]));

			comPortDropDownMenu[c] = new JComboBox();
			for(int i = 0; i < comPorts.length; ++i){
				comPortDropDownMenu[c].addItem(comPorts[i]);
			}
			comPortDropDownMenu[c].addActionListener(this);
			comPortDropDownMenu[c].setActionCommand("comPortDropDownMenu"+c);
			buttons[c+1].add(comPortDropDownMenu[c]);

			/*Connect button*/
			connectBluetooth[c]= new JButton("Connect Bluetooth");
			connectBluetooth[c].setMnemonic(KeyEvent.VK_B);
			connectBluetooth[c].setActionCommand("connectBluetooth"+c);
			connectBluetooth[c].addActionListener(this);
			connectBluetooth[c].setToolTipText("Press to connect Bluetooth (selected serial port)");
			connectBluetooth[c].setEnabled(false);
			buttons[c+1].add(connectBluetooth[c]);



			/*close bluetooth port button*/
			closeBluetooth[c]= new JButton("Close Connection");
			closeBluetooth[c].setMnemonic(KeyEvent.VK_B);
			closeBluetooth[c].setActionCommand("closeBluetooth"+c);
			closeBluetooth[c].addActionListener(this);
			closeBluetooth[c].setToolTipText("Press to close Bluetooth connection (selected serial port)");
			closeBluetooth[c].setEnabled(false);
			buttons[c+1].add(closeBluetooth[c]);
		}

		/*Add selections to the GUI*/
		for (int i = 0; i< buttons.length;++i){
			add(buttons[i]);
		}


		/**Task annotation selection*/
		taskDropDown = new JComboBox();
		String[] tasks = {"Single Task Walk","Motor Task Walk","Dual Task Walk","Motor Task","5 RM Sit To Stand","30 s Sit To Stand","CMJ"};	//Tasks
		for(int i = 0; i < tasks.length; ++i){
			taskDropDown.addItem(tasks[i]);
		}
		taskDropDown.addActionListener(this);
		taskDropDown.setActionCommand("taskDropDown");
		taskDropDown.setEnabled(true);
		/*Add the drop down to the GUI*/
		add(taskDropDown);

		/*Real-time visualization of the captured data*/
		 drawImages = new DrawImage[4];
		 Dimension[] dims = new Dimension[4];
		 for (int i = 0;i<4;++i){
			dims[i] = new Dimension(imWidth/3,imHeight/2);
		 }

		/*Add 3D orientation images*/
		orientationWindow = new VisualizeAxes[1];
		int tempCount = 0;

		for (int i = 0; i< dims.length; ++i){
			drawImages[i] = new DrawImage(dims[i],(int)Math.pow(2d,12d));
			drawImages[i].setOpaque(true);
			add(drawImages[i]);
			drawImages[i].paintImageToDraw();
			if (i == 3 ){
				orientationWindow[tempCount] = new VisualizeAxes(imWidth*2/3,imHeight/2);
				orientationWindow[tempCount].setOpaque(true);
				add(orientationWindow[tempCount]);
				++tempCount;
			}

		}

		/**add WindowListener for closing graciously*/
		parentFrame.addWindowListener(this);
	}

	public void init(){
		/*Reserve pointers*/
		capture = new Capture[1];
		captureThread = new Thread[1];
		portToConnectTo = new String[1];
		serialPort = new SerialPort[1];
		oStream = new BufferedOutputStream[1];
		for (int i = 0; i<portToConnectTo.length;++i){
			portToConnectTo[i]	= null;
			serialPort[i]		= null;
			capture[i]			= null;
			captureThread[i]	= null;
			oStream[i]			= null;
		}

		preferences = Preferences.userRoot().node(this.getClass().getName());
		try{
			savePath = preferences.get(keySP,new File( "." ).getCanonicalPath()); /*Use current working directory as
		default*/
		}catch (IOException ex){
			System.out.println(ex);
			savePath = ".";
		}
		/*Instantiate fileChooser*/
		fileChooser = new JFileChooser(savePath);							/*Implements the file chooser*/
		//fileChooser.setFileSelectionMode(JFileChooser.FILES_ONLY);	/*Limit to choosing files*/
		fileChooser.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
	}

	/**Implement ActionListener*/
	public void actionPerformed(ActionEvent e) {
		/**Select file for saving*/
		if ("chooseSaveFile".equals(e.getActionCommand())){
			int returnVal = fileChooser.showOpenDialog(this.getParent());

            if (returnVal == JFileChooser.APPROVE_OPTION) {
                file = fileChooser.getSelectedFile();

				try{
					if (file.isDirectory()){
						preferences.put(keySP,file.getCanonicalPath());
					}else{
						preferences.put(keySP,(new File(file.getParent())).getCanonicalPath());
					}
					System.out.println("Save path set "+preferences.get(keySP,"."));
				}catch (IOException ex){
					System.out.println(ex);
					savePath = ".";
				}
            } else {
                System.out.println("Cancelled file dialog");
            }

		}

		/**Select com port*/
		if ("comPortDropDownMenu0".equals(e.getActionCommand())){
			portToConnectTo[0] = (String) ((JComboBox)e.getSource()).getSelectedItem();
			connectBluetooth[0].setEnabled(true);
		}

		/**Connect bluetooth*/
		if ("connectBluetooth0".equals(e.getActionCommand())){
			connectBluetooth[0].setEnabled(false);

			/*Connect bluetooth here*/
			serialPort[0] = new SerialPort(portToConnectTo[0]);
			Boolean connectSuccess = false;
			int attempts = 0;
			while (attempts < 5 && !connectSuccess){
				try {
					System.out.println("Port opened: " + serialPort[0].openPort());
					System.out.println("Parameters set: " + serialPort[0].setParams(115200, 8, 1, 0));	/*Serial port has been set to 115200*/
					connectSuccess = true;
				}catch (SerialPortException ex){
					System.out.println(ex);
					++attempts;
					/**Sleep for 2 seconds prior to re-trying*/
					try{
						Thread.sleep(1500);
					}catch (Exception err){
						System.out.println("Could not sleep");
						System.out.println(err);
					}
				}
			}
			if (connectSuccess){
				beginSampling.setEnabled(true);
				closeBluetooth[0].setEnabled(true);
				doBeginSampling0();
			}
			requestFocusInWindow();
		}

		/**Close connection*/
		if ("closeBluetooth0".equals(e.getActionCommand())){
			closeBluetooth[0].setEnabled(false);
			if (capture[0] !=null){
				capture[0].quitCapture();
				try{
					captureThread[0].join();
					capture[0] = null;
					captureThread[0] = null;
				} catch (Exception err){
					System.out.println("Couldn't join captureThread");
				}
			}

			if (serialPort[0] != null) {
				try{
					serialPort[0].purgePort(SerialPort.PURGE_RXABORT
					| SerialPort.PURGE_RXCLEAR | SerialPort.PURGE_TXABORT | SerialPort.PURGE_TXCLEAR);
					serialPort[0].closePort();
					serialPort[0] = null;
				} catch (Exception err){
					System.out.println("Couldn't close port");
				}
			}
		}

		/**Begin sampling*/
		if ("beginSampling".equals(e.getActionCommand())) {
			doBeginSampling();
		}

		/**Tare*/
		if ("tare".equals(e.getActionCommand())){
			if (capture[0] != null){
				capture[0].setZeros();
			}
			requestFocusInWindow();
		}


		/**Indicate task*/
		if ("taskDropDown".equals(e.getActionCommand())){
			currentTask = (short) taskDropDown.getSelectedIndex();
			/*Close previous saveFile and open a new file*/
		}

		if ("endSampling".equals(e.getActionCommand())){
			endSampling.setEnabled(false);
			System.out.println("EndSampling disabled!!");
			continueSampling	= false;
			if (serialPort[0] != null && capture[0] != null){
				closeFile0();
			}
			beginSampling.setEnabled(true);
			chooseSaveFile.setEnabled(true);
			//closeFile();
		}
	}
	/**ActionListener done*/

	private void doBeginSampling(){
		endSampling.setEnabled(true);
		beginSampling.setEnabled(false);
		chooseSaveFile.setEnabled(false);
		continueSampling = true;
		if (file == null){
			file = new File(savePath);
		}

		if (serialPort[0] != null && capture[0] != null){
			closeFile0();
			openFile0();
			capture[0].setOStream(oStream[0]);

		}

		requestFocusInWindow();
	}


	private void doBeginSampling0(){
			if (serialPort[0] != null){
				/*Start a thread for capturing*/
				capture[0] = new CaptureXBIMU(this,serialPort[0],oStream[0],drawImages[0],drawImages[1],drawImages[2],drawImages[3],orientationWindow[0]);
				captureThread[0] = new Thread(capture[0],"captureThread");
				captureThread[0].start();
				System.out.println("Thread started");
			}
	}




	/**Method to initialize the GUI*/
	public static void initAndShowGUI(){
		JFrame f = new JFrame("xBIMU Bluetooth sampling");
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f.add(new XBimuSample(f));
		f.pack();
		Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
		int w;
		int h;
		int traces = 1;
		if (screenSize.width < imWidth+40){w = screenSize.width-40;}else{w=imWidth+40;}
		if (screenSize.height < imHeight*traces+250){h = screenSize.height-40;}else{h=imHeight*traces+250;}
		f.setLocation(20, 20);
		//f.setLocation(screenSize.width/2 - w/2, screenSize.height/2 - h/2);
		f.setSize(w, h);
		f.setVisible(true);
	}


	/**Open a new file for saving*/
	private void openFile0(){
		long dateTime = (new java.util.Date()).getTime();
		System.out.println("Opening a new file "+Long.toString(dateTime));
		if (file != null){
			/**Append timestamp to make the file unique*/
			String fileToOpen = null;
			try{
				fileToOpen = file.getCanonicalPath();
				fileToOpen+="\\"+idArea.getText()+"_xBIMU_";
				fileToOpen+=Long.toString(dateTime);//(date.getTime());
				System.out.println("Trying to open "+fileToOpen);
			}catch (Exception ex){
				System.out.println(ex);
			}
			try{
				/*Finalize any pre-existing outputstream*/
				if (oStream[0] != null){
					oStream[0].flush();
					oStream[0].close();
					oStream[0] = null;
				}
				oStream[0] = new BufferedOutputStream(new FileOutputStream(fileToOpen));
				System.out.println("Opened "+fileToOpen+" for writing");
			}catch (Exception err){
				System.out.println("Couldn't open the file");
			}
		}
	}

	private void closeFile0(){
		if (oStream[0] != null){
			try{
				/*Finalize the outputstream*/
				oStream[0].flush();
				oStream[0].close();
				oStream[0] = null;
			}catch (Exception err){
				System.out.println("Couldn't close the file");
			}
		}
	}

	/**Implement WindowListener*/
	@Override public void 	windowActivated(WindowEvent e){}
	/**Close down the bluetooth, and save the save file*/
	@Override public void 	windowClosed(WindowEvent e){}
	@Override public void 	windowClosing(WindowEvent e){



		for (int i = 0;i<capture.length;++i){
			if (capture[i] != null){
				capture[i].quitCapture();
			}
		}
		continueSampling = false;
		/*Stop the capture thread*/
		for (int i = 0; i<captureThread.length;++i){
			if (captureThread[i] != null){
				System.out.println("Waiting for capture to join");
				try{
					captureThread[i].join();
				} catch (Exception err){
					System.out.println("Couldn't join captureThread");
				}
				System.out.println("Capture joined");
			}
		}

		/*Close the serial ports*/
		for (int i = 0; i<serialPort.length;++i){
			if (serialPort[i] != null){
				try{
					serialPort[i].purgePort(SerialPort.PURGE_RXABORT
					| SerialPort.PURGE_RXCLEAR | SerialPort.PURGE_TXABORT | SerialPort.PURGE_TXCLEAR);
					serialPort[i].closePort();
					serialPort[i] = null;
				} catch (Exception err){
					System.out.println("Couldn't close port");
				}
			}
		}
		System.out.println("All done");
	}
	@Override public void 	windowDeactivated(WindowEvent e){}
	@Override public void 	windowDeiconified(WindowEvent e){}
	@Override public void 	windowIconified(WindowEvent e){}

	/*Start the jogl window */
	@Override public void 	windowOpened(WindowEvent e){
		System.out.println("Window Opened");
		for (int i = 0; i<orientationWindow.length;++i){
			orientationWindow[i].start();
		}
		System.out.println("Started visualizeWindow");
	}


	/**MAIN, just call invokeLater to get the program started*/
	public static void main(String[] args){
		javax.swing.SwingUtilities.invokeLater(new Runnable() {
			public void run(){
				initAndShowGUI();
			}
		}
		);
	}


}