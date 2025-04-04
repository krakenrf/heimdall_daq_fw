#/bin/sh!
echo "Shut down DAQ chain .."
SYSTEM_OS="$(uname -s)"

if [[ "$SYSTEM_OS" == "Darwin" ]];
then
    KILL_SIGNAL=9
else
    KILL_SIGNAL=64
fi

#sudo kill -${KILL_SIGNAL} $(ps aux | grep 'rtl' | awk '{print $2}')
#sudo killall -s ${KILL_SIGNAL} rtl*

sudo pkill -${KILL_SIGNAL} rtl_daq.out
sudo kill -${KILL_SIGNAL} $(ps ax | grep "[p]ython3 _testing/test_data_synthesizer.py" | awk '{print $1}') 2> /dev/null
sudo pkill -${KILL_SIGNAL} sync.out
sudo pkill -${KILL_SIGNAL} decimate.out
sudo pkill -${KILL_SIGNAL} rebuffer.out
sudo pkill -${KILL_SIGNAL} squelch.out
sudo kill -${KILL_SIGNAL} $(ps ax | grep "[p]ython3 _daq_core/delay_sync.py" | awk '{print $1}') 2> /dev/null
sudo kill -${KILL_SIGNAL} $(ps ax | grep "[p]ython3 _daq_core/hw_controller.py" | awk '{print $1}') 2> /dev/null
sudo kill -${KILL_SIGNAL} $(ps ax | grep "[p]ython3 _daq_core/iq_eth_sink.py" | awk '{print $1}') 2> /dev/null
sudo pkill -${KILL_SIGNAL} iq_server.out
