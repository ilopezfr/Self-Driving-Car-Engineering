diff --git a/install-ubuntu.sh b/install-ubuntu.sh
index a285ebe..2d81f19 100755
--- a/install-ubuntu.sh
+++ b/install-ubuntu.sh
@@ -1,14 +1,16 @@
 #! /bin/bash
-sudo apt-get update
-sudo apt-get install git libuv1-dev libssl-dev gcc g++ cmake make
-git clone https://github.com/uWebSockets/uWebSockets 
+apt-get update
+apt-get -y install git libuv1-dev libssl-dev gcc g++ cmake make
+
+git clone https://github.com/uWebSockets/uWebSockets
 cd uWebSockets
 git checkout e94b6e1
 mkdir build
 cd build
 cmake ..
-make 
-sudo make install
+make
+make install
+
 cd ../..
-sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
-sudo rm -r uWebSockets
+ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
+rm -r uWebSockets
