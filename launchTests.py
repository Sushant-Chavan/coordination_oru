import os
import errno
import sys
import subprocess
import argparse
import zipfile
import datetime
import time

def zip_logs(log_dir):
    files_to_zip = []
    log_dir = os.path.abspath(log_dir + "/../") + "/"
    for root, dirs, files in os.walk(log_dir):
        for file in files:
            if not file.endswith(".zip"):
                files_to_zip.append(os.path.join(root, file))

    common_path_prefix = os.path.commonprefix(files_to_zip)

    if len(files_to_zip) > 0:
        curr_time = datetime.datetime.now().strftime("%H:%M:%S %b %d %Y")
        zip_file_path = log_dir + curr_time + '.zip'
        zipf = zipfile.ZipFile(zip_file_path, 'w', zipfile.ZIP_DEFLATED)
        print("Zipping following files to", zip_file_path)
        for f in files_to_zip:
            arc_name = os.path.relpath(f, common_path_prefix)
            zipf.write(f, arc_name)
            print(arc_name)
        zipf.close()
        print("Zipping complete\n")

def clear_logs(log_dir):
    files_to_clear = []
    log_dir = os.path.abspath(log_dir + "/../") + "/"
    for root, dirs, files in os.walk(log_dir):
        for file in files:
            if not file.endswith(".zip"):
                files_to_clear.append(os.path.join(root, file))
    print("Clearing existing log files...")
    for f in files_to_clear:
        if os.path.isfile(f) and (not f.endswith(".zip") and not f.endswith(".dummyLog")):
            os.remove(f)
            print("Deleted file:", f)
    print("Cleared all previous log files\n")

def initialize_test(args):
    log_dir = setup_log_directory(args)
    setup_experienceDB_directory(args)
    zip_logs(log_dir)
    clear_logs(log_dir)

def create_directory_if_needed(dirPath):
    # Make the directory if it does not exist
    try:
        os.makedirs(dirPath)
    except OSError as exc:
        if exc.errno ==errno.EEXIST and os.path.isdir(dirPath):
            pass
        else:
            raise "Could not create directory {}".format(dirPath)

def setup_experienceDB_directory(args):
    sampling_name = "Uniform" if args.no_hotspots else "UsingHotspots"
    kinematics = "ReedsSheep" if args.constrained else "Holonomic"
    planner_names = ["SIMPLE(RRT-Connect)", "Lightning", "Thunder", "EGraphs", "SIMPLE(RRT-Star)"]
    directory = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]  + "/generated/experienceDBs/")
    directory = os.path.join(directory, args.map)
    directory = os.path.join(directory, str(args.nExperiences)+"_TrainingExperiences")
    directory = os.path.join(directory, sampling_name)
    directory = os.path.join(directory, kinematics)
    if args.planner == 3:
        directory = os.path.join(directory, planner_names[args.planner])

    create_directory_if_needed(directory)

def setup_log_directory(args):
    sampling_name = "Uniform" if args.no_hotspots else "UsingHotspots"
    kinematics = "ReedsSheep" if args.constrained else "Holonomic"
    planner_names = ["SIMPLE(RRT-Connect)", "Lightning", "Thunder", "EGraphs", "SIMPLE(RRT-Star)"]
    directory = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]  + "/generated/executionData/")
    directory = os.path.join(directory, args.map)
    directory = os.path.join(directory, planner_names[args.planner])
    directory = os.path.join(directory, str(args.nRobots)+"_Robots")
    directory = os.path.join(directory, kinematics)
    directory = os.path.join(directory, sampling_name)
    directory = os.path.join(directory, str(args.nExperiences)+"_TrainingExperiences")
    directory = os.path.join(directory, "Logs")

    create_directory_if_needed(directory)

    return directory

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--nRobots", type=int, help="Number of robots to be used in the testing. Default: 3", default=3)
    parser.add_argument("--planner", type=int, help="Type of planner to be used for testing (SIMPLE_RRT-Connect: 0, LIGHTNING:1, THUNDER:2, SIMPLE_RRT-Star: 3), Default: Lightning", default=1)
    parser.add_argument("--map", type=str, help="Name of the map used for testing. Default: BRSU_Floor0", default="BRSU_Floor0")
    parser.add_argument("--constrained", type=bool, help="Indicate if the robots are ReedsSheep like vehicles. Default: False (holonomic)", default=False)
    parser.add_argument("--no_hotspots", type=bool, help="Indicate if the experience databases are generated using uniform sampling of the map. Default: False (hotspots used)", default=False)
    parser.add_argument("--nExperiences", type=int, help="Number of training problems used to build the experience DB. Default: 100", default=100)
    parser.add_argument("--nIterations", type=int, help="Number to test iterations to run. Default: 2", default=2)
    parser.add_argument("--timeout", type=int, help="Maximum time allowed for each test in seconds. Default: 300", default=300)
    parser.add_argument("--sleep", type=int, help="Maximum time to pause between iterations in seconds. Default: 10", default=10)
    args = parser.parse_args()

    initialize_test(args)

    run_test_cmd = ["./gradlew", "run", "--offline","-Pdemo=customTests.CustomTesting", "-PnRobots="+str(args.nRobots),
           "-Pplanner="+str(args.planner), "-Pmap="+args.map, "-Pconstrained="+str(int(args.constrained)),
           "-Pno_hotspots="+str(int(args.no_hotspots)), "-Pexp="+str(args.nExperiences)]

    bool_strings = ["False", "True"]

    extract_csv_cmd = ["python3", "generators/logging/LogParser.py", args.map, str(args.planner),
                       "--nRobots="+str(args.nRobots), "--nExperiences="+str(args.nExperiences)]
    if args.constrained:
        extract_csv_cmd.extend(["--constrained="+bool_strings[int(args.constrained)]])
    if args.no_hotspots:
        extract_csv_cmd.extend(["--no_hotspots="+bool_strings[int(args.no_hotspots)]])

    for i in range(args.nIterations):
        if i > 0:
            # Sleep for some time to let all the process close properly before starting next iteration
            print("Waiting {} seconds before starting next iteration...".format(args.sleep))
            time.sleep(args.sleep)
            print("\n\n\n")

        print("=============== Starting test iteration {}/{} ====================".format(i+1, args.nIterations))
        try:
            subprocess.run(run_test_cmd, timeout=args.timeout)
        except subprocess.TimeoutExpired as exc:
            print("Test timed out")

        # Extract the CSV log of all the iterations executed till now
        try:
            subprocess.run(extract_csv_cmd)
        except:
            print("Error in extracting CSV log")

    # Sleep for some time to let all the process close properly
    time.sleep(args.sleep)
    print("All tests completed!")


if __name__ == "__main__":
    main()
