
--------------------------------------------------------------------------
SDRuno weatherFAX plugin
-------------------------------------------------------------------------

![overview](/wfax-example.png?raw=true)

-----------------------------------------------------------------------
weatherfax
-----------------------------------------------------------------------

Even in this time of internet and satelites, short waves are still used
to transmit weathercharts. The picture shows a (noise) signal, 4610 KHz,
with the weatherfax plugin.

The plugin assumes an inputrate of 2000000 / 32 (i.e. 62.5 KHz).
The frequencies on which weathercharts are transmitted are to be found in
e.g.

	https://www.weather.gov/media/marine/rfax.pdf

Most transmissions are preceded by a signal 400 Hz higher than the official
carrier frequency, a transmissions starts with a signal of 300 Hz (Phase
modulated) during a few seconds.

The implementation of the plugin is a reimplementation of the wfax
decoder of the swradio-8 implementation, that decoder is
reimplemented as well.

