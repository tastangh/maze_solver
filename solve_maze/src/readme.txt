Solver.cpp oluşturulur labirent çözecek şekilde 


cmake ‘e aşğıdaki şekilde  cpp’yi tanıtıp executable olarak neyi üreteceğimizi gösteriyoruz.

add_executable(my_solver src/solver.cpp)
target_link_libraries(my_solver ${catkin_LIBRARIES})


Yazılan kodu derleme (her değişiklikte yapıyoruz)
rosnode kill -a   
pkill -9 gzserver
pkill -9 gzclient
pkill -9 roscore 
cd ~/robotlar_ws
catkin_make
source ~/.bashrc


Labirent ortamını simülasyonunu başlatma 
roslaunch micromouse_maze micromouse_maze4.launch 

kamerayı robota göre ayarlıyoruz gazeboda 
oluşturulan executable ı çalıştırma  ve o sırada gözlemleme 

rosrun solve_maze my_solver
    

    simülasyon zamanını 10 katına çıkarmak içni terminale girilir 
gz physics -u 10000


gazebo kapama 

rosnode kill -a   
pkill -9 gzserver
pkill -9 gzclient
pkill -9 roscore 
