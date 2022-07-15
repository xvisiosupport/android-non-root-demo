# Connect the device using adb over network
# adb tcpip 5555
# adb connect 192.168.1.xxx:555
# adb root
# adb remount

libdir="/vendor/lib64"
APP_ABI="arm64-v8a" 
echo "Deploy for ${APP_ABI} target"
for i in libs/${APP_ABI}/*;
do
	adb push $i $libdir
done
for i in bin/${APP_ABI}/*;
do
	adb push $i /vendor/bin
done
exit 0
