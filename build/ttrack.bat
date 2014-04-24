rem Changes only for this environment
setlocal 
@echo off
set PATH="%PATH%;C:\Users\max\projects\opencv\build32\install\x86\vc12\bin;"
set PATH="%PATH%;C:\Program Files (x86)\Boost\lib32-msvc-12.0\;"
set PATH="%PATH%;C:\Program Files\tbb_v4.2\bin\ia32\vc12\;"

"C:\Program Files (x86)\Microsoft Visual Studio 12.0\Common7\IDE\devenv.exe" ttrack.sln