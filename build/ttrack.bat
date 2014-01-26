rem Changes only for this environment
setlocal 
@echo off
set PATH="%PATH%;C:\Users\max\projects\opencv\build32\install\x86\vc10\bin;"
set PATH="%PATH%;C:\Program Files (x86)\Boost\lib32-msvc-10.0\;"
set PATH="%PATH%;C:\Program Files\tbb_v4.2\bin\ia32\vc10\;"

"C:\Program Files (x86)\Microsoft Visual Studio 10.0\Common7\IDE\devenv.exe" ttrack.sln