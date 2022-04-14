#!/bin/sh
echo "Start unit testing.."
echo "Internal warnings are ignored"

rm _testing/test_logs/*.log 2> /dev/NULL
rm _testing/test_logs/*.html 2> /dev/NULL

# Start unit test for the rebuffer module
#sudo python3 -W ignore -m unittest -v _testing/unit_test/test_rebuffer.py

# Start unit test for the decimator module
sudo python3 -W ignore -m unittest -v _testing/unit_test/test_decimator.py

# Start unit test for the squelch module
#sudo python3 -W ignore -m unittest -v _testing/unit_test/test_squelch.py

# Start unit test for the delay synchronizer module
#sudo python3 -W ignore -m unittest -v _testing/unit_test/test_delay_sync.py

# Start unit test for the iq server module - Not Implemented
#sudo python3 -W ignore -m unittest -v _testing/unit_test/test_iq_server.py

#Start system level testing
#sudo python3 -m unittest -v _testing/unit_test/test_sys.py
#./daq_stop.sh