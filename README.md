依赖pcl:1.10  ros  yaml-cpp
安装ros自带pcl,鱼香ros一键安装
wget http://fishros.com/install -O fishros && . fishros
安装yaml-cpp
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build && cd build
cmake -D BUILD_SHARED_LIBS=ON ..
make -j8
sudo make install

git clone https://github.com/echoand/line_laser.git
cd line_laser/build
cmake ..
make 
./main
roscore
cd ..
cd bag
rosbag play -l xian_lasder.bag
rosrun rviz rviz
rviz中Fixed Frame  改为laser_frame，点击Add和By topic添加/ground话题的点云和/wall话题的点云和/scan原始点云
