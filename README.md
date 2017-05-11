# px4plot
MATLAB/Octave m-files for visualizing px4 flight log

How to use
-------------------
The function `px4plot` visualizes major data by reading px4log:

    px4plot('sample.px4log');

Following example is to view roll angle manually:

    log = decode_px4log('sample.px4log');
    [Roll t_Roll] = getLogData(log, 'ATT.Roll');
    plot(t_Roll, Roll);
