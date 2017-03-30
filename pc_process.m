 
rosinit('http://localhost:11311')

sub = rossubscriber('/vslam/pc2',rostype.sensor_msgs_PointCloud2);
pc=receive(sub);
scatter3(pc);