# Connect the device using adb over network
# adb tcpip 5555
# adb connect 192.168.1.xxx:555
# adb root
# adb remount

APP_ABI=x86_64
echo "Deploy for ${APP_ABI} target"
for i in libs/${APP_ABI}/*;
do
	b=$(basename $i)
	if [[ "$b" =~ ^lib.* ]]; then
		libdir="/vendor/lib"
		adb push $i $libdir
	else
		adb push $i /vendor/bin
	fi;
done
exit 0
