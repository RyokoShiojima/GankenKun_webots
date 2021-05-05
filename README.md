# GankenKun_webots

RoboCup2021関連の情報

https://github.com/citbrains/GankenKun_webots/wiki/RoboCup2021%E9%96%A2%E9%80%A3%E6%83%85%E5%A0%B1  

## 環境構築

Webots用の環境構築scriptの内容を実行する。 
 
### CMakeを3.18以上にする。参考https://askubuntu.com/questions/829310/how-to-upgrade-cmake-in-ubuntu  
cd /opt/  
sudo wget https://github.com/Kitware/CMake/releases/download/v3.20.2/cmake-3.20.2-linux-x86_64.sh  
sudo chmod +x /opt/cmake-3.20.2-linux-x86_64.sh  
sudo bash cmake-3.20.2-linux-x86_64.sh  
cd  
sudo ln -s /opt/cmake-3.20.2-linux-x86_64/bin/* /usr/local/bin  

### protocol buffersをインストールする  
以下方法  
wget https://github.com/protocolbuffers/protobuf/releases/download/v3.15.8/protobuf-cpp-3.15.8.tar.gz   
tar -xvzf protobuf-cpp-3.15.8.tar.gz   
解凍した所に移動   
./configure   
make -j(cpuのコア数+1)  
make check   
sudo make install   
sudo ldconfig   
 
 

### .bashrcに以下を追記 
export LD_LIBRARY_PATH=~/citbrains_humanoid/for2050/src/vision/protobuf  
  
### citbrains_humanoidのブランチをuse_picture_from_webotsに変更し以下の作業を行う  
HOMEディレクトリ内で 
cd citbrains_humanoid/for2050/src  
make clean  
cmake .
ccmake .  
USE_VREP_SIMULATOR=OFF  
USE_WEBOTS_SIMULATOR=ONにする  
c e q を順番に押す  
make -j(cpuのコア数+1)  
make install  

### このリポジトリをクローンする  
git checkout hr46_b3m_webots  
cd GankenKun_webots/controllers/hr46_b3m  
cmake .
make -j5

https://github.com/citbrains/GankenKun_webots/wiki/RoboCup2021%E9%96%A2%E9%80%A3%E6%83%85%E5%A0%B1  

環境構築の方法  
https://github.com/citbrains/GankenKun_webots/wiki/webots%E3%81%A7%E8%A9%A6%E5%90%88%E3%81%AE%E7%92%B0%E5%A2%83%E3%82%92%E4%BD%9C%E6%88%90%E3%81%99%E3%82%8B%E6%89%8B%E9%A0%86  

![image](https://user-images.githubusercontent.com/5755200/115998122-cc332400-a620-11eb-90d5-0e83166787e8.png)


