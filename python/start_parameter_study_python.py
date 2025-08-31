import os
import subprocess
import multiprocessing
import time
import itertools
import json

env = os.environ.copy()

# Define the directory path and command components
directory_path = os.getcwd()
command_base =  os.path.join(directory_path,"build/Release/bin/Release/3L-VehicleRoutingApplication.exe")
output_folder = r"C:\Users\mahu123a\Documents\Data\ParameterStudy\NoClassifier/"
input_folder = os.path.join(directory_path,"data/input/3l-cvrp/gendreau_parameterstudy/")
parameter_files = [os.path.join(directory_path,"data/input/3l-cvrp/parameters/Parameters_NoClassifier.json")]
temporary_parameter_files = r"C:\Users\mahu123a\Documents\Data\ParameterStudy/Parameters"
number_of_processes = 6
MAX_TIME_SEC = 700

PARAMETER_STUDY = {"IteratedLocalSearchParams": {"LimitNoImpr" :  [5,8,12],
                   "K_RandomMoves" : [3,5,8],
                   "PerturbationTypes": [["K_RandomInsertions","K_RandomSwaps"],["K_RandomSwaps","K_RandomInsertions"]],
                    "LocalSearchTypes": [["TwoOpt","IntraSwap","IntraInsertion","InterSwap","InterInsertion","DeleteEmptyRoutes"],["InterSwap","InterInsertion","TwoOpt","IntraSwap","IntraInsertion","DeleteEmptyRoutes"]],
                    "RoundsWithNoImprovement":  [1,3,5,10]}}

# Navigate to the directorys

def create_new_parameter_file(file_path_base_parameter:str,
                              folder_path_save_temp_parameters:str,
                              parameter_study_dict:dict,
                              parameter_group:str,
                              counter_file_name:int) -> str:
    
    # Open and read the JSON file
    with open(file_path_base_parameter, 'r') as file:
        data = json.load(file)

    filename = "Parameters_"+str(counter_file_name)+".json"

    for key,value in parameter_study_dict.items():
        data[parameter_group][key] = value

    save_path = os.path.join(folder_path_save_temp_parameters,filename)


    with open(save_path, "w") as f:
        json.dump(data, f, indent=4)

    return save_path


def add_tasks(task_queue, folder_path=input_folder):
    # List all files in the folder
    file_list = os.listdir(folder_path)
    #Select base parameter file
    counter = 0
    for parameter_file in parameter_files:

        #Identify each change of parameter groups
        for param_group in PARAMETER_STUDY:
            keys, values = zip(*(PARAMETER_STUDY[param_group].items()))
            parameter_study_dict_of_dicts = [dict(zip(keys, v)) for v in itertools.product(*values)]

            #Iterate over all products of different parameter studies 
            for parameter_study_dict in parameter_study_dict_of_dicts:
                parameter_file_path = create_new_parameter_file(parameter_file, temporary_parameter_files, parameter_study_dict, param_group,counter)
                counter += 1
                for seed_offset in range(3):  # seeds 0, 1, 2
                    for file_name in file_list:
                        full_path = os.path.join(folder_path, file_name)
                        if not os.path.isfile(full_path):
                            continue
                        else:
                            task_queue.put((file_name, parameter_file_path, seed_offset))

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
        print(f"Timeout: {filename} with seed {seed_offset} and parameters {parameter_file} exceeded {MAX_TIME_SEC} seconds.\n")

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

    for file in os.listdir(temporary_parameter_files): 
        file_path = os.path.join(temporary_parameter_files,file)
        if os.path.exists(file_path): 
            os.remove(file_path)

    end_time = time.time()
    elapsed = end_time - start_time
    print(f"\nTotal subprocesses executed: {counter.value} in {elapsed} seconds")

if __name__ == "__main__":
    run()

