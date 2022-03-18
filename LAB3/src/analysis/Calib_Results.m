% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 763.432774768849981 ; 761.425951985444840 ];

%-- Principal point:
cc = [ 516.449723136553075 ; 369.684507851574438 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.000000000000000 ; -0.000000000000000 ; -0.001122567617889 ; 0.004500140967540 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 7.436820802062361 ; 7.121677696130860 ];

%-- Principal point uncertainty:
cc_error = [ 9.023604820148941 ; 6.679568765719257 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.000000000000000 ; 0.000000000000000 ; 0.003389600460142 ; 0.004265000218664 ; 0.000000000000000 ];

%-- Image size:
nx = 1008;
ny = 756;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 7;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 0 ; 0 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -2.100304e+00 ; -2.057395e+00 ; 2.144121e-01 ];
Tc_1  = [ -1.422025e-01 ; -6.967956e-02 ; 2.670179e-01 ];
omc_error_1 = [ 9.424427e-03 ; 8.425763e-03 ; 1.810637e-02 ];
Tc_error_1  = [ 3.191417e-03 ; 2.471534e-03 ; 3.299721e-03 ];

%-- Image #2:
omc_2 = [ -1.965902e+00 ; -1.947685e+00 ; 6.234918e-01 ];
Tc_2  = [ -1.198642e-01 ; -7.114723e-02 ; 3.086062e-01 ];
omc_error_2 = [ 1.005673e-02 ; 8.045553e-03 ; 1.616819e-02 ];
Tc_error_2  = [ 3.691782e-03 ; 2.776540e-03 ; 3.073910e-03 ];

%-- Image #3:
omc_3 = [ -1.943870e+00 ; -2.013270e+00 ; 1.183656e-02 ];
Tc_3  = [ -1.196368e-01 ; -5.946026e-02 ; 2.501332e-01 ];
omc_error_3 = [ 7.856354e-03 ; 9.658820e-03 ; 1.634055e-02 ];
Tc_error_3  = [ 2.979299e-03 ; 2.297276e-03 ; 3.037240e-03 ];

%-- Image #4:
omc_4 = [ -1.796874e+00 ; -1.775695e+00 ; 7.326366e-01 ];
Tc_4  = [ -1.299815e-01 ; -7.536270e-02 ; 3.466363e-01 ];
omc_error_4 = [ 1.046674e-02 ; 8.687062e-03 ; 1.473858e-02 ];
Tc_error_4  = [ 4.194973e-03 ; 3.152040e-03 ; 3.226731e-03 ];

%-- Image #5:
omc_5 = [ -1.903825e+00 ; -1.738268e+00 ; -2.019204e-01 ];
Tc_5  = [ -1.242691e-01 ; -3.009023e-02 ; 2.384883e-01 ];
omc_error_5 = [ 7.380889e-03 ; 9.942123e-03 ; 1.476516e-02 ];
Tc_error_5  = [ 2.833247e-03 ; 2.194719e-03 ; 2.931496e-03 ];

%-- Image #6:
omc_6 = [ 1.842911e+00 ; 1.465170e+00 ; -3.384968e-01 ];
Tc_6  = [ -1.138590e-01 ; -6.358587e-02 ; 2.779379e-01 ];
omc_error_6 = [ 7.595927e-03 ; 9.206266e-03 ; 1.447512e-02 ];
Tc_error_6  = [ 3.305478e-03 ; 2.505200e-03 ; 3.110374e-03 ];

%-- Image #7:
omc_7 = [ 2.105574e+00 ; 2.071249e+00 ; 3.238992e-01 ];
Tc_7  = [ -8.200971e-02 ; -7.376692e-02 ; 1.971720e-01 ];
omc_error_7 = [ 8.696179e-03 ; 7.802173e-03 ; 1.700189e-02 ];
Tc_error_7  = [ 2.458899e-03 ; 1.818071e-03 ; 2.380484e-03 ];

