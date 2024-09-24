# LpLoRa 

This battery testing branch is for SG internal battery testing only, which:

- transmits every 15s at SF12, BW125, CR45, Tx power +22dBm, LDRO ON;
- after every transmission, go to Rx for 15s;
- ...and transmit again afterwards.

The packet it sends is a 24 byte packet, including:

- 15 bytes of unique ID of the uC, 
- 1 byte of `RFEOLF` bit (for low battery detection), 
- 4 bytes of transmission counter (+1 after each Tx)
- 4 bytes of CRC32-MJPEG2 hash covers the first 20 bytes.

## Code quality for this branch

Yea I know the code quality is shit. But I don't care, I just want to test the battery. 

Please go to somewhere else if you expect Safe & sound Rust code.
