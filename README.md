# rnx2dat
This code is to convert rinex navigation file to dat file, now gps is supported 
It is based on open source project -- rtklib
I just added the functions which convert nav data to binary that can be sent by satellite.
code is simple.

chmod +x build
./build

you can get the executable file

./a.out rinex_file toe mins
rinex_file --> rinex file name
toe --> the time period you want to convert
mins --> how long the binary file you want

such as 
./a.out 2015.p 439200 3

this is Initial version, maybe too simple