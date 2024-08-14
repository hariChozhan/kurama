import sys
import subprocess
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout

class RemoteLaunchThread(QThread):
    def __init__(self, remote_host, remote_user, parent=None):
        super().__init__(parent)
        self.remote_host = remote_host
        self.remote_user = remote_user
        
    def run(self):
        # Command to launch the ROS 2 launch file on the remote system (System B)
        command = [
            'ssh', f'{self.remote_user}@{self.remote_host}', 
            'source /opt/ros/your_ros2_distro/setup.bash && ros2 launch your_package your_launch_file.py'
        ]
        
        # Use subprocess to execute the command over SSH
        subprocess.Popen(command)
        print(f"Launch file executed on {self.remote_host}!")

class MyApp(QWidget):
    def __init__(self):
        super().__init__()
        
        # Set up the window
        self.setWindowTitle('ROS 2 Remote Launch Example')
        self.setGeometry(100, 100, 300, 100)
        
        # Create a button
        self.button = QPushButton('Launch ROS 2 Remotely', self)
        self.button.clicked.connect(self.launch_ros2)  # Connect button click to callback
        
        # Set up the layout
        layout = QVBoxLayout()
        layout.addWidget(self.button)
        self.setLayout(layout)
    
    def launch_ros2(self):
        # Remote system (System B) credentials
        remote_host = '192.168.1.10'  # Replace with System B's IP address
        remote_user = 'your_username' # Replace with System B's username
        
        # Start the remote ROS 2 launch file in a separate thread
        self.thread = RemoteLaunchThread(remote_host, remote_user)
        self.thread.start()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    myApp = MyApp()
    myApp.show()
    sys.exit(app.exec_())
