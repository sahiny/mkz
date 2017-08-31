Lcs = data.Alpha_L1.Data\-data.Fy_L1.Data
Rcs = data.Alpha_R1.Data\-data.Fy_R1.Data

Tcs = 0.5*(data.Alpha_L1.Data + data.Alpha_R1.Data)\-(data.Fy_L1.Data + data.Fy_R1.Data)

clf; hold on
plot(0.5*abs(data.Alpha_L1.Data + data.Alpha_R1.Data), abs(data.Fy_L1.Data + data.Fy_R1.Data))
plot(xlim, xlim*Tcs)