import socket
import time
import numpy as np
import click
import threading
import struct

val = 0.0
def send_udp_data(ip: str, port: int, message: str, frequency: float):
    global val
    # Create a UDP socket
    sender_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"Sending UDP data to {ip}:{port}...")
    try:
        while True:
            # Send message to the specified IP and port
            val += 0.1
            # Send a struct containing two floats
            sender_sock.sendto(struct.pack('ff', val, val), (ip, port))

            time.sleep(1 / frequency)
    finally:
        sender_sock.close()


def receive_udp_data(recv_ip: str, recv_port: int):
    # Create a UDP socket for receiving
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Bind the socket to the specified IP and port for receiving
    recv_sock.bind((recv_ip, recv_port))

    print(f"Listening for UDP data on {recv_ip}:{recv_port}...")


    # List to store arrival times
    arrival_times = []

    try:
        while True:
            # Receive data from the socket
            data, addr = recv_sock.recvfrom(1024)  # buffer size is 1024 bytes
            if len(data) == 8:
                # Unpack the data
                left, right = struct.unpack('ff', data)
                print(f"Received wheel velocities from {addr}: Left = {left}, Right = {right}")
            else:
                print(f"Received unexpected data size from {addr}: {data}")


            # Record the current time
            arrival_times.append(time.time())

            # Compute statistics if at least two messages have been received
            if len(arrival_times) > 1:
                # Calculate intervals between message arrivals
                intervals = np.diff(arrival_times)

                # Frequency is the reciprocal of the average interval
                recv_frequency = 1 / np.mean(intervals)

                # Standard deviation of the intervals
                std_dev = np.std(intervals)

                print(f"Receiving Frequency: {recv_frequency:.2f} messages/second, Standard Deviation: {std_dev:.2f} seconds")

    except KeyboardInterrupt:
        print("UDP receiver terminated.")
    finally:
        recv_sock.close()
    return None

# Take IP and port as command line arguments
@click.command()
@click.option("--recv_ip", default="192.168.0.15", help="IP address to listen on")
@click.option("--recv_port", default=8888, help="Port to listen on")
@click.option("--send_ip", help="IP address to send data to")
@click.option("--send_port", default=8888, help="Port to send data to")
@click.option("--message", default="Hello", help="Message to send")
@click.option("--frequency", default=30.0, help="Frequency of sending messages in Hz")
def main(recv_ip: str, recv_port: int, send_ip: str, send_port: int, message: str, frequency: float):

    # Start the sending thread
    send_thread = threading.Thread(target=send_udp_data, args=(send_ip, send_port, message, frequency), daemon=True)
    send_thread.start()

    receive_udp_data(recv_ip, recv_port)

    # Kill the sending thread
    send_thread.join()



# Example usage
if __name__ == "__main__":
    main()
