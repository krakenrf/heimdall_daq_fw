#/bin/sh!
echo "Shut down DAQ chain .."
#sudo kill -64 $(ps aux | grep 'rtl' | awk '{print $2}')
#sudo killall -s 9 rtl*

sudo pkill -64 rtl_daq.out
sudo kill -64 $(ps ax | grep "[p]ython3 _testing/test_data_synthesizer.py" | awk '{print $1}') 2> /dev/null
sudo pkill -64 sync.out
sudo pkill -64 decimate.out
sudo pkill -64 rebuffer.out
sudo kill -64 $(ps ax | grep "[p]ython3 _daq_core/delay_sync.py" | awk '{print $1}') 2> /dev/null
sudo kill -64 $(ps ax | grep "[p]ython3 _daq_core/hw_controller.py" | awk '{print $1}') 2> /dev/null
sudo kill -64 $(ps ax | grep "[p]ython3 _daq_core/iq_eth_sink.py" | awk '{print $1}') 2> /dev/null
sudo pkill -64 iq_server.out
