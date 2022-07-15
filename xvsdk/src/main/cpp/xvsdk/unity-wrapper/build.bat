SET CURRENTDIR="%cd%"
mkdir autobuild
cd autobuild
SET AUTOBUILD=%cd%

mkdir msvc2019
cd msvc2019

mkdir build-x64
cd build-x64
cmake -G "Visual Studio 16 2019" -A x64 ^
-DCMAKE_INSTALL_PREFIX="%AUTOBUILD%/install/msvc2019-x64/" ^
-DXSlamSDK_DIR="C:\xvisio\fast_slam\autobuild\install\msvc2019-x64" ^
../../..
cmake --build . --config Release --target install
cd %AUTOBUILD%/msvc2019

mkdir build-x86
cd build-x86
cmake -G "Visual Studio 16 2019" -A Win32 ^
-DCMAKE_INSTALL_PREFIX="%AUTOBUILD%/install/msvc2019-x86/" ^
-DXSlamSDK_DIR="C:\xvisio\fast_slam\autobuild\install\msvc2019-x86" ^
../../..
cmake --build . --config Release --target install

cd %AUTOBUILD%/
mkdir msvc2017
cd msvc2017

mkdir build-x64
cd build-x64
cmake -G "Visual Studio 15 2017 Win64" ^
-DCMAKE_INSTALL_PREFIX="%AUTOBUILD%/install/msvc2017-x64/" ^
-DXSlamSDK_DIR="C:\xvisio\fast_slam\autobuild\install\msvc2017-x64" ^
../../..
cmake --build . --config Release --target install
cd %AUTOBUILD%/msvc2017

mkdir build-x86
cd build-x86
cmake -G "Visual Studio 15 2017" ^
-DCMAKE_INSTALL_PREFIX="%AUTOBUILD%/install/msvc2017-x86/" ^
-DXSlamSDK_DIR="C:\xvisio\fast_slam\autobuild\install\msvc2017-x86" ^
../../..
cmake --build . --config Release --target install
cd %CURRENTDIR%


cd %CURRENTDIR%

pause
