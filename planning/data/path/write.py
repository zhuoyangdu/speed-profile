import math
flag = 'cycle'

if flag == 'L7L6':
    path_file = open('L7L6.txt', 'w')
    for i in range(0,1000):
        log_path = "502.5\t%f\t0\t%f\n" %(i,i)
        path_file.write(log_path)
    path_file.close()

if flag == 'L7L2':
    path_file = open('L7L2.txt', 'w')
    for i in range(0,495):
        log_path = '502.5\t%f\t0\t%f\n' %(i,i)
        path_file.write(log_path)

    for i in range(0, 90, 5):
        x = 495+7.5*math.cos(i/180.0*math.pi)
        y = 495+7.5*math.sin(i/180.0*math.pi)
        theta = -i/180.0*math.pi
        print i,x,y,theta
        log_path = '%f\t%f\t%f\t1000\n' %(x,y,theta)
        path_file.write(log_path)

    for i in range(495,0,-1):
        log_path = '%f\t502.5\t-1.570\t%f\n' %(i,i)
        path_file.write(log_path)
    path_file.close()

if flag == 'cycle':
    path_file = open('cycle.txt','w')
    for i in range(0,100):
        log_path = '0\t%f\t0\t0\n' %(i-50)
        path_file.write(log_path)

    path_file.close()

