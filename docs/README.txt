This devices plays back the activity of another device as it was recorded by the
Phy in a previous simulation.

Note: Unlike "ext_2G4_device_playback" it feeds from v2 Phy channel activity dumps/traces.

It takes as command line arguments:
  -inputf=<inputfile>: Path and begining of the dump files names to be played
                       back. For example, the Tx file will be <inputfile>.Tx.csv
  -txoff : Do not send Tx requests
  -rxoff : Do not send Rx requests
  -rssioff : Do not send RSSI requests
  -ccaoff : Do not send CCA requests
