#Open the file containing data from the imu
data_file = open("data.txt","r")

#create a matrix, and add the data for the 4? axes
data = [];
for line in data_file:
	data.append(line.split(","))

#initialize the averages for each axis
x_avg = y_avg = z_avg = w_avg = 0

#calculate the averages
for val in data:
	x_avg+=float(val[0])
	y_avg+=float(val[1])
	z_avg+=float(val[2])
	w_avg+=float(val[3])
x_avg/=len(data)
y_avg/=len(data)
z_avg/=len(data)
w_avg/=len(data)

#create a matrix containing the averages, where the index for an axis corresponds to the index of its values in data[]
avg_mat = [x_avg,y_avg,z_avg,w_avg]

#initialize the covariance matrix
q=[[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]

#calculate the covariance
for i in range(0,4):
	for j in range(0,4):
		for val in data:
			q[i][j]+= (float(val[i])-avg_mat[i])*(float(val[j])-avg_mat[j])
		q[i][j]/=(len(data)-1)

print("x_avg = " + str(x_avg) + "\ny_avg = " + str(y_avg) + "\nz_avg = " + str(z_avg)+ "\nw_avg = " + str(w_avg))
for r in q:
	print(r)
