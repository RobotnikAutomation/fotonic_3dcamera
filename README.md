# fotonic_3dcamera
Driver for the Fotonic TOF Camera

To install the library:
cd fotonic_3dcamera/external/fz-linux-api-130322/fz-linux-api_x64/fz_api_src
make
sudo cp libFZ_API.* /usr/local/lib

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
