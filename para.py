import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel
import paramiko

class RobotControlGUI(QWidget):
    def __init__(self):
        super().__init__()

        # Set up the GUI
        self.initUI()

    def initUI(self):
        # Window settings
        self.setWindowTitle('Robot Control Interface')
        self.setGeometry(100, 100, 300, 150)

        # Create a layout
        layout = QVBoxLayout()

        # Create a label
        self.status_label = QLabel('Status: Disconnected', self)
        layout.addWidget(self.status_label)

        # Create a connect button
        self.connect_button = QPushButton('Connect', self)
        self.connect_button.clicked.connect(self.connect_button_pressed)
        layout.addWidget(self.connect_button)

        # Set layout to the main window
        self.setLayout(layout)

    def start_remote_launch(self, ip, username, password, launch_file):
        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(ip, username=username, password=password)

            # Construct the command to launch the ROS 2 package
            command = f'source /opt/ros/humble/setup.bash && ros2 launch {launch_file}'
            stdin, stdout, stderr = ssh.exec_command(command)

            # Optional: read stdout and stderr to check for errors
            output = stdout.read().decode('utf-8')
            error = stderr.read().decode('utf-8')
            ssh.close()

            if error:
                self.status_label.setText(f'Error: {error}')
                print(f'Error: {error}')
            else:
                self.status_label.setText(f'Successfully started {launch_file}')
                print(output)

        except Exception as e:
            self.status_label.setText(f'Connection failed: {str(e)}')
            print(f'Connection failed: {str(e)}')

    def connect_button_pressed(self):
        self.status_label.setText('Status: Connecting...')
        
        # Start sensor package on controller 1
        self.start_remote_launch('192.168.1.2', 'user1', 'password1', 'controller1_launch.py')

        # Start robot control package on controller 2
        self.start_remote_launch('192.168.1.3', 'user2', 'password2', 'controller2_launch.py')
        
        self.status_label.setText('Status: Connected')

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = RobotControlGUI()
    gui.show()
    sys.exit(app.exec_())
