clear all
clc;

%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%speed of light = 3e8
%% User Defined Range and Velocity of target
% *%TODO* :
% define the target's initial position and velocity. Note : Velocity
% remains contant
d0 = 110;         %initial position 
v0 = -28;        %initial velocity 


%% FMCW Waveform Generation

% *%TODO* :
%Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.


%Operating carrier frequency of Radar 
fc= 77e9;             %carrier freq
r_res = 1;
c = 3e8;
Rmax = 200;

Bsweep = c/(2*r_res); % Sweep bandwidth
Tchirp = 5.5*2*Rmax/c; % Chirp time
slope = Bsweep/Tchirp; % slope of a single chirp
                                                          
%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation. 
Nd=128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp. 
Nr=1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t=linspace(0,Nd*Tchirp,Nr*Nd); %total time for samples


%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal

%Similar vectors for range_covered and time delay.
r_t=zeros(1,length(t));
td=zeros(1,length(t));


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    
    
    % *%TODO* :
    %For each time stamp update the Range of the Target for constant velocity. 
    r_t(i) = d0 + v0*t(i);
    td(i) = 2*r_t(i)/c; 
    % *%TODO* :
    %For each time sample we need update the transmitted and
    %received signal. 
    Tx(i) = cos(2*pi*(fc*t(i) + slope*t(i)^2/2));
    Rx(i) = cos(2*pi*(fc*(t(i) - td(i)) + slope*(t(i) - td(i))^2/2));
    % *%TODO* :
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i) * Rx(i);
    
end

%% RANGE MEASUREMENT


 % *%TODO* :
%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.
Mix = reshape(Mix,[Nr,Nd]);
 % *%TODO* :
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
sig_fft1 = fft(Mix,Nr);  
sig_fft1 = sig_fft1./Nr;
 % *%TODO* :
% Take the absolute value of FFT output
sig_fft1 = abs(sig_fft1);  
 % *%TODO* :
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
single_side_sig_fft1 = sig_fft1(1:Nr/2);

%plotting the range
figure1 = figure ('Name','Range from First FFT');
subplot(2,1,1)

 % *%TODO* :
 % plot FFT output 
plot(single_side_sig_fft1); 
title('Range from First FFT')
xlabel('Range in meters') 
ylabel('Signal Strength') 
xticks(0:20:200);
yticks(0:0.05:0.5);
axis ([0 200 0 0.5]);

saveas(figure1, 'Output_Images/Range_First_FFT.png')

%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM


% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure2 = figure('Name','Range Doppler Map from Second FFT');
surf(doppler_axis,range_axis,RDM);
title('Range Doppler Map from Second FFT')
xlabel('Doppler Axis') 
ylabel('Range Axis')
zlabel('RDM value') 
saveas(figure2, 'Output_Images/Range_Doppler_MAP_Second_FFT.png')

%% CFAR implementation

%Sliding Window comprising of CUT,Guard Cells and Training Cells through the complete Range Doppler Map

% *%TODO* :
%Configuring the sliding window dimensions
%Select the number of Training Cells in both dimensions Tcr:Row and Tcd:Colums
Tcr = 9;
Tcd = 8;
% *%TODO* :
%Select the number of Guard Cells in both dimensions Gcr:Row and Gcd:Colums around the Cell under 
%test (CUT) for accurate estimation
Gcr = 5;
Gcd = 5;
% *%TODO* :
% offset the threshold by SNR value in dB
offset = 1.4;

% *%TODO* :

%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.


% Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
% CFAR






%Normalising the RDM Matrix
RDM = RDM/max(max(RDM));

% The row number of the CUT(Cell under test) will vary from Tcr+Gcr+1(Training band rows + Guard band rows + 1) as First CUT will be located in this row
% It will go till (Nr/2)-(Gcr+Tcr); subtracting by Nr/2 as the that is the last rows number in the RDM matrix
% Because while Taking just one side of signal from Range dimension. FFT2 was calculated for half of the sample (Positive returns) 
% sig_fft2 = sig_fft2(1:Nr/2,1:Nd)
for i = Tcr+Gcr+1:(Nr/2)-(Gcr+Tcr)
    % Similarly column number of the CUT cell will vary from Tcd+Gcd+1(Training band columns + Guard band columns + 1) as first column 
    % It will go till Nd-(Gcd+Tcd); subtracting by Nd(Number of chirps in the signal Mixed signal) as that is the last column number in the RDM matrix
    for j = Tcd+Gcd+1:Nd-(Gcd+Tcd)
        
       % Vector to store noise_level for each iteration on training cells
        noise_level = zeros(1,1);
        
        % Calculate noise SUM in the area around CUT excluding the Guard cells area 
        
        % while calculating the noise iterating is done from the left extreme end: i-(Tcr+Gcr) to the right extreme end: i+(Tcr+Gcr) of the sliding window 
        % surrounding the current CUT in consideration: i
        for p = i-(Tcr+Gcr) : i+(Tcr+Gcr)
            for q = j-(Tcd+Gcd) : j+(Tcd+Gcd)
                % Excluding the cells lying in the Guard cells area of the sliding window 
                if (abs(i-p) > Gcr || abs(j-q) > Gcd)
                    % converting the RDM value back to db as earlier it was stored as power of the signal using
                    % RDM = 10*log10(RDM) above while collecting the RDM using 2nd FFT
                    noise_level = noise_level + db2pow(RDM(p,q));
                end
            end
        end
        
        % Convert the averaged noise for the RDM above to db so that;  offset in dbs can directly be added and Adaptive threshold could 
        % calculated for the particular sliding window position in the RDM 
        threshold = pow2db(noise_level/(2*(Tcd+Gcd+1)*2*(Tcr+Gcr+1)-(Gcr*Gcd)-1));
        threshold = threshold + offset;
        CUT = RDM(i,j);
        
        if (CUT < threshold)
            RDM(i,j) = 0;
        else
            RDM(i,j) = 1;
        end
    end
end 

% *%TODO* :
% The process above will generate a thresholded block, which is smaller 
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0. 
RDM(union(1:(Tcr+Gcr),end-(Tcr+Gcr-1):end),:) = 0;  % Rows
RDM(:,union(1:(Tcd+Gcd),end-(Tcd+Gcd-1):end)) = 0;  % Columns 


% *%TODO* :
%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
figure3 = figure('Name','CA-CFAR Filtered RDM');
surf(doppler_axis,range_axis,RDM);
colorbar;
title('CA-CFAR Filtered RDM with only Target and no Clutter')
xlabel('Doppler Axis') 
ylabel('Range Axis')
zlabel('RDM value') 
saveas(figure3, 'Output_Images/CA-CFAR_Filtered_RDM.png')


 
 