import threading
import time

# Create events for each thread
event2 = threading.Event()

# Create a flag to control Thread 1
thread1_flag = threading.Event()

def thread1_function():
    last_time = time.time()
    while True:
        print("Thread 1 is running")
        event2.set()
        time.sleep(1)
        # print("")
        if time.time() - last_time > 3:
            time.sleep(2)
            print("Thread 1.1 is running")
            last_time = time.time()


def thread2_function():
    while True:
        event2.wait()
        print("Thread 2 is running")
        # event2.clear()

def thread3_function():
    while True:
        event2.wait()
        print("Thread 3 is running")
        # event2.clear()

def thread4_function():
    while True:
        event2.wait()
        print("Thread 4 is running")
        event2.clear()

# # Create and start the threads
# thread1 = threading.Thread(target=thread1_function)
# thread2 = threading.Thread(target=thread2_function)
# thread3 = threading.Thread(target=thread3_function)
# thread4 = threading.Thread(target=thread4_function)

# thread1.start()
# thread2.start()
# thread3.start()
# thread4.start()

# # Initially, allow only Thread 1 to run
# event2.set()

# # Wait for all threads to finish (you may need to manually stop the threads)
# thread1.join()
# thread2.join()
# thread3.join()
# thread4.join()

queue = []
queue.append(1)

for i in range(10):
    queue.append(i)
    print(queue.pop(0))
