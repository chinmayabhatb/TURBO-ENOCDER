%#This example how data is framed and encoded in Lte system this can be compared with internal LTE tool box so that 
% how many data is differ from behavioral and HDL processing
% But the frame structre can be done before it then can give to to
% encoder so that encoded data is in frame stuctre
% Transimissin bit is generated with random function
%CODEDATA is encoded data plus 4 bits of trail bits which can be ignored
%during decoding
%As per digital communication After encoder that COdeData is serialized and
%sent over network .

%% 
clc;
clear all;
close all;
% Generate input data frames. Generate reference encoded data using |lteTurboEncode|.

turboframesize = 40;
numframes = 2;

txBits    = cell(1,numframes);
codedData = cell(1,numframes);

for ii = 1:numframes
    txBits{ii} = randi([0 1],turboframesize,1);
    codedData{ii} = turbo_encoder(txBits{ii});
    %plotting values
    figure;
    plot(codedData{ii});
    title(['Frame ',num2str(ii),' CodeData']);
end
    

%TURBO ENCODER
function [EncodedOut]= turbo_encoder (InMessage)
K=length(InMessage);
P = get_interleaver('random', K);
inv_P(P) = 1:length(P);
inter_seq.P = P;
inter_seq.inv_P = inv_P;

u = InMessage;

u_intrl = u(P);
x_p = get_RSC_encoded_seq(u);
x_q = get_RSC_encoded_seq(u_intrl);

%#adjusting last 4 tail bits and control to [0,0]
K=K+4;
x_p(K) = mod(u(K-5) + x_p(K-4) + x_p(K-5), 2);
u(K) = x_p(K);
u(K-1) = mod(u(K) + u(K-2) + x_p(K-2), 2);
u(K-2) = mod(u(K) + u(K-3) + x_p(K-3), 2);
u(K-3) = mod(u(K) + u(K-4) + x_p(K-4), 2);
x_p(K-1) = mod(u(K-1) + u(K-3) + x_p(K-2) + x_p(K-3), 2);
x_p(K-2) = mod(u(K-2) + u(K-4) + x_p(K-3) + x_p(K-4), 2);
x_p(K-3) = mod(u(K-3) + u(K-5) + x_p(K-4) + x_p(K-5), 2);
x_q(K) = mod(u_intrl(K-5) + x_q(K-4) + x_q(K-5), 2);
u_intrl(K) = x_q(K);
u_intrl(K-1) = mod(u_intrl(K) + u_intrl(K-2) + x_q(K-2), 2);
u_intrl(K-2) = mod(u_intrl(K) + u_intrl(K-3) + x_q(K-3), 2);
u_intrl(K-3) = mod(u_intrl(K) + u_intrl(K-4) + x_q(K-4), 2);
x_q(K-1) = mod(u_intrl(K-1) + u_intrl(K-3) + x_q(K-2) + x_q(K-3), 2);
x_q(K-2) = mod(u_intrl(K-2) + u_intrl(K-4) + x_q(K-3) + x_q(K-4), 2);
x_q(K-3) = mod(u_intrl(K-3) + u_intrl(K-5) + x_q(K-4) + x_q(K-5), 2);
X_P=x_p';
X_Q=x_q';
% u(K+1:K+4,1)=0;
% X_P(K+1:K+4,1)=0;
% X_Q(K+1:K+4,1)=0;
x = [u;X_P;X_Q];
x=int8(x);
EncodedOut=x;
end 

function P = get_interleaver(type, param)
K = param;
P = zeros(1,K);
if strcmp(type, 'def')
    P(1) = 12;
    for i = 2:K
        P(i) = mod(13*(P(i-1)-1) + 7, K) + 1;
    end
elseif strcmp(type, 'random')
%     We will be using random interleaver but it may not give interleaving
%     sequence always, so we will be tracking it and re-perform the
%     construction if loop gets struck. We will be using a maximum of 5
%     trials
    S = 4;    
    trial_len = 5;
    trial_ind = 1;
    success_flag = 0;
    while(trial_ind < trial_len)
        sim_track = 0;
        T = 1:K;
        i = 1;
        while(1)
            if sim_track > length(T),
                break
            end
            ind = randi(length(T), 1);
            x = T(ind);
            j = max(i-S, 1):i-1;
            if sum(abs(ones(1,length(j))*x - P(j)) > S) == length(j)
                T(T == x) = [];
                P(i) = x;
                i = i + 1;
                sim_track = 0;
            end
            if i > K
                success_flag = 1;
                break;
            end                
            sim_track = sim_track + 1;
        end
        if success_flag,
            break
        end
    end
end
end

function x = get_RSC_encoded_seq(u)

% using RSC encoder with G(D) = [         1 + D^2     ]
%                               | 1,    -----------   |
%                               [       1 + D + D^2   ]

K = length(u);
x = zeros(1,K);

state = [0,0];

for i = 1:K
    x(i) = mod(state(2) + u(i), 2);
    state = [mod(u(i) + x(i), 2), mod(state(1) + x(i), 2)];
end
end
