import os
import subprocess
import sys
import getpass

# Function to install local requirements (requirements-local.txt)
def install_local_requirements():
    try:
        print("Installing local requirements...")
        # Path to the local requirements file
        script_dir = os.path.dirname(os.path.realpath(__file__))
        local_reqs = os.path.join(script_dir, "requirements-local.txt")

        # Install the requirements using pip
        result = subprocess.run(["pip", "install", "-r", local_reqs], capture_output=True, text=True)

        if result.returncode == 0:
            print("Local requirements installed successfully!")
        else:
            print("Error installing local requirements:")
            print(result.stderr)
            sys.exit(1)  # Exit if local requirements fail to install

    except Exception as e:
        print(f"Error during local installation: {e}")
        sys.exit(1)  # Exit if an error occurs

# Function to install requirements on Raspberry Pi via SSH
def install_requirements_ssh():
    # Import paramiko only after ensuring local requirements are installed
    import paramiko

    # Get Raspberry Pi connection details
    username = input("Enter Raspberry Pi username: ")
    password = getpass.getpass("Enter Raspberry Pi password: ")

    raspberry_pi_ip = input("Enter Raspberry Pi IP address: ")

    # Path to the remote requirements file (on Raspberry Pi)
    script_dir = os.path.dirname(os.path.realpath(__file__))
    angela_reqs = os.path.join(script_dir, "requirements-angela.txt")

    try:
        print(f"Connecting to Raspberry Pi at {raspberry_pi_ip}...")

        # Create SSH client instance
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())  # Automatically accept new host keys

        # Connect to the Raspberry Pi
        client.connect(raspberry_pi_ip, username=username, password=password)

        # Run the command to install the requirements on Raspberry Pi
        print("Installing requirements on Raspberry Pi (Angela's setup)...")
        command = f"pip install -r {angela_reqs}"
        stdin, stdout, stderr = client.exec_command(command)

        # Output the result of the installation
        print(stdout.read().decode())
        err = stderr.read().decode()
        if err:
            print(err)

        # Log message after successful installation
        print("Angela's requirements installed successfully on Raspberry Pi!")

        # Close the SSH connection
        client.close()

    except Exception as e:
        print(f"Error during SSH connection or installation: {e}")
        sys.exit(1)  # Exit if an error occurs

# Main script execution
if __name__ == '__main__':
    print("Starting the configuration process...")

    # Step 1: Install local requirements
    install_local_requirements()

    # Step 2: Install requirements on Raspberry Pi via SSH
    install_requirements_ssh()

    # All done!
    print("Configuration completed successfully!")
