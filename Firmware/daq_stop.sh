#/bin/sh!
echo "Shut down DAQ chain .."
sudo kill $(ps aux | grep 'rtl' | awk '{print $2}')
sudo killall -s 9 rtl*

sudo pkill rtl_daq.out
sudo kill $(ps ax | grep "[p]ython3 _testing/test_data_synthesizer.py" | awk '{print $1}') 2> /dev/null
sudo pkill sync.out
sudo pkill decimate.out
sudo pkill rebuffer.out
sudo pkill squelch.out
sudo kill $(ps ax | grep "[p]ython3 _daq_core/delay_sync.py" | awk '{print $1}') 2> /dev/null
sudo kill $(ps ax | grep "[p]ython3 _daq_core/hw_controller.py" | awk '{print $1}') 2> /dev/null
sudo pkill iq_server.out


