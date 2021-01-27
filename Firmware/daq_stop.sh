#/bin/sh!
echo "Shut down DAQ chain .."
sudo kill $(ps aux | grep 'rtl' | awk '{print $2}')
sudo killall -s 9 rtl*

sudo pkill rtl_daq.out
sudo pkill sync.out
sudo pkill decimate.out
sudo pkill rebuffer.out
sudo pkill squelch.out
sudo pkill python3
sudo pkill iq_server.out


