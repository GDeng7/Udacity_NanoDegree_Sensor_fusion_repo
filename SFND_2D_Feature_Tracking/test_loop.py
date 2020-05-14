import os
import subprocess

def listToString(s):  
    str1 = ""  
    
    for ele in s:  
        str1 += ele   
    
    return str1  

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
		if cnt == 1:
			TableHead =  "|No. | Detector + Descriptor | Total Keypoints | Total Matches points | Total Time (ms) | \n"
			Segments = "|:---:|:-----:|:-----:|:-----:|:-----:|\n"
			OutputResults.append(TableHead + Segments);

		OutputResults.append("|" + str(cnt)+ " " + output_string)

with open("../Result/Test_results.txt", "w") as f:
	f.write(listToString(OutputResults))