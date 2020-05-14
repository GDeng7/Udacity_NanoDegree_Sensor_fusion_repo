import os
import subprocess

if not os.path.exists("Result"):
	os.makedirs("Result")

os.chdir("./build")

detector_list = ["SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"]
descriptor_list = ["BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"]

cnt = 0
OutputResults = []
for detector in detector_list:
	for descriptor in descriptor_list:
		cnt += 1
		print ('Called No. %d Detector: %s Descriptor: %s' % (cnt, detector, descriptor))
		state, output_string = subprocess.getstatusoutput('./2D_feature_tracking %s %s' % (detector, descriptor))
		OutputResults.append(str(cnt) + "\n" + output_string+"\n")
		print(len(output_string))
		
final_result = "======================================\n".join(OutputResults)
with open("../Result/Test_results.txt", "w") as f:
	f.write(final_result)