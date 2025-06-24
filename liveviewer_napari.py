import numpy as np
import serial
import multiprocessing
from multiprocessing import shared_memory
import os
import datetime
import time
import napari
import tifffile as tf
from qtpy.QtCore import QTimer
from qtpy.QtWidgets import QPushButton, QWidget, QVBoxLayout, QCheckBox, QLineEdit, QFileDialog, QFormLayout,  QDoubleSpinBox
#os.environ["QT_SCREEN_SCALE_FACTORS"] = "1.5"  #per-screen scaling to 1.5 (typical for laptops)
STANDARD_SAVE_DIR = r'C:\Users\dan20\OneDrive - Johann Wolfgang Goethe Universität\Module\Masterarbeit\Aufnahmen\Tests'
COM_PORT = 'COM5'
BAUDRATE = 921600
X_RESOLUTION = 512
Y_RESOLUTION = 512
BUFFER_SIZE = (2, X_RESOLUTION, Y_RESOLUTION) 
BIDIRECTIONAL = False
SHARED_MEMORY_NAME = 'shared_nparray'
NP_DTYPE = np.uint16

class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s
    def readline(self):
        """Non-blocking serial readline implementation."""
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            if data == b'':
                data = b"\n"
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

class ImagingParameterForm(QWidget):
    def __init__(self):
        super().__init__()

        layout = QFormLayout()

        '''
        self.zoom_input = QDoubleSpinBox()
        self.zoom_input.setRange(0.6, 40.0)
        self.zoom_input.setValue(1.0)
        self.zoom_input.setToolTip("Zoom factor of the scan")
        layout.addRow("Zoom:", self.zoom_input)

        self.pixel_dwell_input = QDoubleSpinBox()
        self.pixel_dwell_input.setRange(6.0, 100.0)
        self.pixel_dwell_input.setValue(10.0)
        self.pixel_dwell_input.setSuffix(" µs")
        self.pixel_dwell_input.setToolTip("Time per pixel in microseconds")
        layout.addRow("Pixel Dwell Time:", self.pixel_dwell_input)

        self.pixel_pitch_input = QLineEdit()
        self.pixel_pitch_input.setPlaceholderText("e.g. 0.5")
        self.pixel_pitch_input.setToolTip("Pixel pitch in micrometer")
        layout.addRow("Pixel Pitch (µm):", self.pixel_pitch_input)

        self.objective_input = QComboBox()
        self.objective_input.addItems(["10x NA 0.3", "32x NA 0.85", "63x NA 1.3"])
        self.objective_input.setToolTip("Objective")
        layout.addRow("Objective:", self.objective_input)

        self.resolution_input = QComboBox()
        self.resolution_input.addItems(["256", "512", "1024"])
        self.resolution_input.setToolTip("Resolution in pixels. IMPORTANT: Must be changed in the Teensy code as well!")
        layout.addRow("Resolution:", self.resolution_input)
        '''

        self.pump_power_input = QDoubleSpinBox()
        self.pump_power_input.setRange(10.0, 400.0)
        self.pump_power_input.setValue(50.0)
        #self.pump_power_input.setSuffix(" mW")
        self.pump_power_input.setToolTip("Pump laser power in mW")

        self.pump_wavelength_input = QLineEdit()
        self.pump_wavelength_input.setPlaceholderText("e.g., 800")
        self.pump_wavelength_input.setToolTip("Pump laser wavelength in nm")

        self.stokes_power_input = QDoubleSpinBox()
        self.stokes_power_input.setRange(10.0, 400.0)
        self.stokes_power_input.setValue(100.0)
        #self.stokes_power_input.setSuffix(" mW")
        self.stokes_power_input.setToolTip("Stokes laser power in mW")

        self.path_input = QLineEdit("")
        self.path_input.setToolTip("Path where image files will be saved")
        self.path_input.setText(STANDARD_SAVE_DIR)
        self.browse_button = QPushButton("Browse...")
        self.browse_button.clicked.connect(self.select_save_path)
        
        layout.addRow("Pump Power (mW):", self.pump_power_input)
        layout.addRow("Pump Wavelength (nm):", self.pump_wavelength_input)
        layout.addRow("Stokes Power (mW):", self.stokes_power_input)
        
        layout.addRow("Save Path:", self.path_input)
        layout.addRow("", self.browse_button)
        self.setLayout(layout)

    def select_save_path(self):
        path = QFileDialog.getExistingDirectory(self, "Select Save Directory")
        if path:
            self.path_input.setText(path)
        self.browse_button = QPushButton("Browse...")
        self.browse_button.clicked.connect(self.select_save_path)

    
def read_serial(port, baudrate, frame_queue, stop_event, shm_name):
    """Read serial data and places full frames into a queue for saving"""
    buffer_index = 0  #Toggle between buffer 0 and 1
    serial_port = serial.Serial(port, baudrate, timeout=1)
    serial_port.set_buffer_size(rx_size=10000000)
    custom_readline = ReadLine(serial_port)
    shared_image = shared_memory.SharedMemory(create=False, name=shm_name)
    image_buffer = np.ndarray(BUFFER_SIZE, dtype=NP_DTYPE, buffer=shared_image.buf)
    image_buffer.fill(0)
    while not stop_event.is_set():
        reading = custom_readline.readline()
        data = reading.decode("ASCII").strip("<>\r\n").split(',')
        if data[0] == 'f': #New frame detected
            frame_queue.put(buffer_index)
            buffer_index = 1 - buffer_index
            #image_buffer[buffer_index].fill(0)
            continue
        try:
            pixel_x, pixel_y, pixel_value = int(data[0]), int(data[1]), int(data[2])
            image_buffer[buffer_index,pixel_y,pixel_x] = pixel_value
        except ValueError:
            print(f"[WARNING] Corrupt data: {data} - Skipping...")
        except IndexError:
            print(f"[INFO] Waiting for frame out signal... {data}")
    serial_port.close()
    frame_queue.put(buffer_index) #save last frame before stopping
    frame_queue.put(None)
    print('[INFO] Read process stopped.')

def read_serial_live(port, baudrate, vis_queue, stop_event, shm_name):
    serial_port = serial.Serial(port, baudrate, timeout=1)
    serial_port.set_buffer_size(rx_size=10000000)
    custom_readline = ReadLine(serial_port)
    shared_image = shared_memory.SharedMemory(create=False, name=shm_name)
    image_buffer = np.ndarray(BUFFER_SIZE, dtype=NP_DTYPE, buffer=shared_image.buf)
    while not stop_event.is_set():
        reading = custom_readline.readline()
        try:
            data = reading.decode("ASCII").strip("<>\r\n").split(',')
            if data[0] == 'f': #New frame detected
                vis_queue.put(1)
                continue
            pixel_x, pixel_y, pixel_value = int(data[0]), int(data[1]), int(data[2])
            image_buffer[0,pixel_y,pixel_x] = pixel_value
        except ValueError:
            print(f"[WARNING] Corrupt data: {data} - Skipping...")
        except IndexError:
            print(f"[INFO] Waiting for frame out signal... {data}")
        except AttributeError:
            print(f"[INFO] No new signal... {data}")
    serial_port.close()
    print('[INFO] Read process stopped.')

def save_frames(q, vis_q, shm_name, bidirectional, metadata):
    """Saves frames from the queue to CSV or tif files"""
    shared_image = shared_memory.SharedMemory(create=False, name=shm_name)
    image_buffer = np.ndarray(BUFFER_SIZE, dtype=NP_DTYPE, buffer=shared_image.buf)
    frame_number = 0
    while True:
        buffer_index = q.get()
        if buffer_index is None:
            break
        if frame_number == 0:
            print('[INFO] Beginning Acquisition...')
            frame_number+=1
            continue
        if bidirectional == True:
            image_buffer[buffer_index,:,::2]=np.flip(image_buffer[buffer_index,:,::2], axis=1)
        tf.imwrite(f'frame_{frame_number}.tif', image_buffer[buffer_index], metadata=metadata, compression = None)
        print(f"[INFO] Frame {frame_number} saved!")
        vis_q.put((frame_number, buffer_index))
        frame_number+=1
    print('[INFO] Save process stopped.')

def update_viewer():
    """update the Napari viewer"""
    global current_buffer, images
    if not visualization_queue.empty():
        if buttonContinuousLive.isChecked() == False:
            frame_number, buffer_index = visualization_queue.get()
            current_buffer = 1-buffer_index
            if frame_number == 1:
                images.data = np.copy(image_buffer[np.newaxis,buffer_index,:X_RESOLUTION,:Y_RESOLUTION])#Add first frame to data with new axis to concatenate frames on
            elif checkboxBidirectional.isChecked() == False:
                images.data = np.concatenate((images.data, np.copy(image_buffer[np.newaxis,buffer_index,:X_RESOLUTION,:Y_RESOLUTION])), axis=0)
            else:
                images.data = np.concatenate((images.data, np.copy(image_buffer[np.newaxis,buffer_index,:X_RESOLUTION,:Y_RESOLUTION])), axis=0)
            if viewer.dims.point[0] >= images.data.shape[0] - 2:#only jump to newest frame if slider already on the last newest frame  
                viewer.dims.set_point(0, images.data.shape[0] - 1)
            images.refresh()
        else: 
            images.data = np.copy(image_buffer[0,:X_RESOLUTION,:Y_RESOLUTION])
            if checkboxBidirectional.isChecked() == True:
                images.data[:,::2] = np.flip(images.data[:,::2], axis=1)
            images.refresh()

def start_acquisition():
    print('[INFO] Initializing Acquisition...')
    viewer.status = '[INFO] Initializing Acquisition...'
    global images, acquisition_counter, metadata, resolution
    acquisition_counter+=1
    save_dir = param_form.path_input.text()
    try:
        os.chdir(save_dir)#set directory
    except FileNotFoundError:
        print(f'[ERROR] Directory {save_dir} not found! Please select a valid path.')
    folder_name=str(datetime.datetime.now())[:19].replace(':', '-')    
    try:
        os.mkdir(folder_name)
        print(f'[INFO] Created Folder: {folder_name} at directory {save_dir}')
    except FileExistsError:
        print('[WARNING] Directory already exists!')
    os.chdir(folder_name)    
    save_dir=f'{save_dir}/{folder_name}'
    try:#create folder to save into
        os.mkdir(f'{save_dir}/Acquisition {acquisition_counter}')
        os.chdir(f'{save_dir}/Acquisition {acquisition_counter}')
    except FileExistsError:
        print('[WARNING] Acquisition Directory already exists!')
    images = viewer.add_image(np.zeros(BUFFER_SIZE, dtype=NP_DTYPE)[np.newaxis,0,:,:], name=f'Acquisition {acquisition_counter}')
    stop_event.clear()
    read_process = multiprocessing.Process(target=read_serial, args=(COM_PORT, BAUDRATE, frame_queue, stop_event, SHARED_MEMORY_NAME,), daemon=True)
    read_process.start()
    metadata = {
        'axes' : 'XY',
        #'Zoom': param_form.zoom_input.value(),
        #'Pixel Dwell Time': param_form.pixel_dwell_input.value(),
        #'Pixel Pitch': param_form.pixel_pitch_input.text(),
        'Pump Power': param_form.pump_power_input.value(),
        'Pump Wavelength': param_form.pump_wavelength_input.text(),
        'Stokes Power': param_form.stokes_power_input.value(),
        #'Objective': param_form.objective_input.currentText(),
        'Date': str(datetime.datetime.now())[:19].replace(':', '-'),
        }
    save_process = multiprocessing.Process(target=save_frames, args=(frame_queue, visualization_queue, SHARED_MEMORY_NAME, checkboxBidirectional.isChecked(), metadata), daemon=True)
    save_process.start()
    buttonStartAcquisition.setVisible(False)
    buttonContinuousLive.setVisible(False)
    buttonStopAcquisition.setVisible(True)
    param_form.setEnabled(False)
    timer.start(10)#start Qtimer

def stop_acquisition():
    stop_event.set()
    buttonStopAcquisition.setVisible(False)
    buttonStartAcquisition.setVisible(True)
    buttonContinuousLive.setVisible(True)
    param_form.setEnabled(True)
    metadata['axes'] = 'XYZ'
    tf.imwrite(f'Acquisition_{acquisition_counter}.tif', images.data, metadata=metadata, compression = None)
    print("[INFO] Saved acquired image sequence.")
    print("[INFO] Acquisition stopped.")
    viewer.status = '[INFO] Acquisition stopped.'
    time.sleep(0.02)
    timer.stop()#stop Qtimer

def toggleContinuosLive():
    if buttonContinuousLive.isChecked() == True:
        print('[INFO] Initializing Continuous Live ...')
        viewer.status = '[INFO] Initializing Continuous Live...'
        global images, acquisition_counter
        acquisition_counter+=1
        images = viewer.add_image(np.zeros(BUFFER_SIZE, dtype=NP_DTYPE)[np.newaxis,0,:,:], name=f'Acquisition {acquisition_counter}')
        stop_event.clear()
        read_process = multiprocessing.Process(target=read_serial_live, args=(COM_PORT, BAUDRATE, visualization_queue, stop_event, SHARED_MEMORY_NAME,), daemon=True)
        read_process.start()
        buttonStartAcquisition.setVisible(False)
        buttonContinuousLive.setStyleSheet("background-color : red")
        param_form.setEnabled(False)
        timer.start(10)#start Qtimer
        print('[INFO] Continuous Live Mode activated!')
        viewer.status = '[INFO] Continuous Live Mode activated!'

    else:
        timer.stop()#stop Qtimer
        stop_event.set()
        buttonStopAcquisition.setVisible(False)
        buttonStartAcquisition.setVisible(True)
        param_form.setEnabled(True)
        while not visualization_queue.empty():#clear queue for next acquisition
            visualization_queue.get()
        print("[INFO] Acquisition stopped.")
        viewer.status = '[INFO] Acquisition stopped.'
        buttonContinuousLive.setStyleSheet("background-color: blue;")
        print('[INFO] Continuous Live Mode deactivated!')
        viewer.status = '[INFO] Continuous Live Mode deactivated!'

if __name__ == "__main__": 
    #Initialize Napari viewer
    viewer = napari.Viewer(title='Napari: LiveViewer SRS Acquisition')
    #create shared memory between processes for image data
    shared_memory_size = int(np.dtype(NP_DTYPE).itemsize * np.prod(BUFFER_SIZE))
    shared_array_memory = shared_memory.SharedMemory(create=True, size=shared_memory_size, name='shared_nparray')
    image_buffer = np.ndarray(BUFFER_SIZE, dtype=NP_DTYPE, buffer=shared_array_memory.buf)
    image_buffer.fill(0)
    current_buffer = 0

    #Image Parameter Form
    param_form = ImagingParameterForm()
    viewer.window.add_dock_widget(param_form, area='left', name='Acquisition Settings')

    #Add Button to switch to continuos live mode
    buttonContinuousLive = QPushButton('Continuous Live')
    buttonContinuousLive.setStyleSheet("background-color: blue;")
    buttonContinuousLive.setCheckable(True)
    buttonContinuousLive.setChecked(False)
    #viewer.window.add_dock_widget(buttonContinuousLive, area='left')
    buttonContinuousLive.toggled.connect(toggleContinuosLive)
    
    #Add Button to start the acquisition process
    buttonStartAcquisition = QPushButton('Start Acquisition')
    buttonStartAcquisition.setStyleSheet("background-color : green") 
    acquisition_counter = 0
    buttonStartAcquisition.clicked.connect(start_acquisition)
    #viewer.window.add_dock_widget(buttonStartAcquisition, area='left')
    
    #Add Button to stop the acquisition process
    buttonStopAcquisition = QPushButton('Stop Acquisition')
    buttonStopAcquisition.setStyleSheet("background-color : red") 
    buttonStopAcquisition.clicked.connect(stop_acquisition)
    buttonStopAcquisition.setVisible(False)

    #Add checkbox for bidirectional mode
    checkboxBidirectional = QCheckBox('Bidirectional')
    checkboxBidirectional.setChecked(False)

    #Add elements to a single dock widget
    acq_control = QWidget()
    acq_layout = QVBoxLayout()
    acq_layout.addWidget(checkboxBidirectional)
    acq_layout.addWidget(buttonStartAcquisition)
    acq_layout.addWidget(buttonContinuousLive)
    acq_layout.addWidget(buttonStopAcquisition)
    acq_control.setLayout(acq_layout)
    viewer.window.add_dock_widget(acq_control, name='Acqusition Control Panel', area='left')

    #Label frames according to slider postion
    viewer.text_overlay.visible = True
    viewer.dims.axis_labels = ['Frames', 'x', 'y']
    def update_slider(event):
        frame = viewer.dims.current_step[0]
        viewer.text_overlay.text = f"Frame: {frame+1}"
    viewer.dims.events.current_step.connect(update_slider)

    #Declare multiprocessing events, queues and processes
    stop_event = multiprocessing.Event()
    frame_queue = multiprocessing.Queue()
    visualization_queue = multiprocessing.Queue()

    #Qt timer to update the image layer every x milliseconds (non-blocking)
    timer = QTimer()
    timer.timeout.connect(update_viewer)

    #start Napari GUI
    napari.run()

    #Terminate all processes and release memory after Napari is closed
    print('[INFO] Closed Napari. Cleaning Up...')
    if not stop_event.is_set():
        stop_event.set()
    shared_array_memory.close()
    shared_array_memory.unlink()