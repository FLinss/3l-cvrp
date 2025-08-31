import os
import subprocess
import multiprocessing
import time
import random

env = os.environ.copy()

# Define the directory path and command components
directory_path = os.getcwd()
command_base =  os.path.join(directory_path,"build/Release/bin/Release/3L-VehicleRoutingApplication.exe")
output_folder = r"C:\Users\mahu123a\Documents\Data\ParameterStudy\NoClassifier/"
input_folder = os.path.join(directory_path,"data/input/3l-cvrp/gendreau/")
parameter_file_list_path = r"C:\Users\mahu123a\Documents\3l-cvrp-heuristic\data\input\3l-cvrp\Non_executed_parameter_files"
number_of_processes = 6
MAX_TIME_SEC = 900

# Navigate to the directorys

def add_tasks(task_queue, folder_path=input_folder):

    # List all files in the folder
    parameter_file_list = os.listdir(parameter_file_list_path)
    for file in parameter_file_list:  # seeds 0, 1, 2
        full_path = os.path.join(parameter_file_list_path, file)
        instance_name = file.split("0000")[-1].split("Parameters-")[-1]
        print(instance_name)
        seed_offset = random.randint(0, 2)
        if not os.path.isfile(full_path):
            continue
        task_queue.put((instance_name, full_path, seed_offset))

    return task_queue

def run_Heuristic_exe(filename, parameter_file, seed_offset, counter, lock):

    command = [
        command_base,
        "-i", input_folder,
        "-f", filename,
        "-o", output_folder,
        "-p", parameter_file,
        "-s", str(seed_offset)
    ]

    try:
        print(f"Starting: {filename} with seed {seed_offset}\n")
        print(f"Command: {command}\n")

        subprocess.run(command, check=True, capture_output=True, cwd=directory_path, env=env, timeout=MAX_TIME_SEC)

        print(f"Finished: {filename} with seed {seed_offset}\n")

        with lock:
            counter.value += 1

    except subprocess.CalledProcessError as e:
        print(f"Failed: {filename} with seed {seed_offset}")
        print(e)
        print(f"Return code: {e.returncode}")
        print("STDERR:", e.stderr.decode())

    except subprocess.TimeoutExpired:
        print(f"Timeout: {filename} with seed {seed_offset} and parameters {parameter_file} exceeded 800 seconds.\n")

def process_tasks(task_queue, counter, lock):
    while not task_queue.empty():
        try:
            filename, parameter_file, seed_offset = task_queue.get_nowait()
        except:
            break  # In case of race condition
        run_Heuristic_exe(filename, parameter_file, seed_offset, counter, lock)
    return True

def run():
    empty_task_queue = multiprocessing.Queue()
    full_task_queue = add_tasks(empty_task_queue)

    #Create shared counter and lock
    counter = multiprocessing.Value('i', 0)
    lock = multiprocessing.Lock()
    start_time = time.time()

    processes = []
    for _ in range(number_of_processes):
        p = multiprocessing.Process(target=process_tasks, args=(full_task_queue,counter, lock))
        processes.append(p)
        p.start()

        time.sleep(1)

    for p in processes:
        p.join()

    end_time = time.time()
    elapsed = end_time - start_time
    print(f"\nTotal subprocesses executed: {counter.value} in {elapsed} seconds")

if __name__ == "__main__":
    run()

