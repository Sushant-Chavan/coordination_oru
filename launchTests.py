import os
import sys
import subprocess
import argparse
import zipfile
import datetime
import time

def zip_logs(curr_dir):
    path_to_logs = "generated/experienceLogs/"
    files_to_zip = []
    for root, dirs, files in os.walk(path_to_logs):
        for file in files:
            if os.path.splitext(file)[1] == ".log":
                files_to_zip.append(os.path.join(root, file))

    if len(files_to_zip) > 0:
        curr_time = datetime.datetime.now().strftime("%H:%M:%S %b %d %Y")
        zip_file_path = curr_dir + path_to_logs + curr_time + '.zip'
        zipf = zipfile.ZipFile(zip_file_path, 'w', zipfile.ZIP_DEFLATED)
        print("Zipping following files to", zip_file_path)
        for f in files_to_zip:
            zipf.write(f)
            print(f)
        zipf.close()
        print("Zipping complete\n")

def clear_logs():
    path_to_logs = "generated/experienceLogs/"
    contents = os.listdir(path_to_logs)
    print("Clearing existing log files...")
    for item in contents:
        f  = os.path.join(path_to_logs, item)
        if os.path.isfile(f) and (not f.endswith(".zip") and not f.endswith(".dummyLog")):
            os.remove(f)
            print("Deleted file:", f)
    print("Cleared all previous log files\n")

def initialize_test():
    curr_dir = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]) + "/"
    zip_logs(curr_dir)
    clear_logs()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--nRobots", type=int, help="Number of robots to be used in the testing. Default: 3", default=3)
    parser.add_argument("--planner", type=int, help="Type of planner to be used for testing (SIMPLE_RRT-Connect: 0, LIGHTNING:1, THUNDER:2, SIMPLE_RRT-Star: 3), Default: Lightning", default=1)
    parser.add_argument("--map", type=str, help="Name of the map used for testing. Default: BRSU_Floor0", default="BRSU_Floor0")
    parser.add_argument("--nIterations", type=int, help="Number to test iterations to run. Default: 2", default=2)
    parser.add_argument("--timeout", type=int, help="Maximum time allowed for each test in seconds. Default: 300", default=300)
    parser.add_argument("--sleep", type=int, help="Maximum time to pause between iterations in seconds. Default: 10", default=10)
    args = parser.parse_args()

    initialize_test()

    run_test_cmd = ["./gradlew", "run", "-Pdemo=customTests.CustomTesting", "-PnRobots="+str(args.nRobots),
           "-Pplanner="+str(args.planner), "-Pmap="+args.map]
    extract_csv_cmd = ["python3", "generators/logging/LogParser.py", args.map, str(args.planner)]

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
