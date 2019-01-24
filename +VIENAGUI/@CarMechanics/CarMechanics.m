% Copyright (c) 2019 F. Ferreira da Silva
%                    João F. P. Fernandes
%                    Bruno Tibério
%
% Permission is hereby granted, free of charge, to any person obtaining a
% copy of this software and associated documentation files (the
% "Software"), to deal in the Software without restriction, including
% without limitation the rights to use, copy, modify, merge, publish,
% distribute, sublicense, and/or sell copies of the Software, and to permit
% persons to whom the Software is furnished to do so, subject to the
% following conditions:
%
% The above copyright notice and this permission notice shall be included
% in all copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
% OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
% MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
% NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
% DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
% OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
% USE OR OTHER DEALINGS IN THE SOFTWARE.

classdef CarMechanics
    %CARMECHANICS car model to simulate its driving
    
    properties
        motor = VIENAGUI.InductionMotor;
    end
    
%--------------------------------------------------------------------------
%-PROPERTIES---------------------------------------------------------------
%--------------------------------------------------------------------------
    
    properties (Constant)
        %Car Properties (FIAT Seicento)
        carMass = 700; %[kg]
        length = 3.337 %[m]
        frontArea = 1.508*1.420;  %[m^2]
        
        %Wheel
        wheelMass = 3; %[kg]
        wheelRadius = 0.28; %[m]
        wheelInertia = 1/2 * wheelMass * wheelRadius^2; %[Kg m^2]
        rollFriction = 0.01; 
        cornerStiff = 60; %not used
        
        %Air Drag
        dragCoeff = 0.33;
%         airTemperature = 273; %[K]
%         airPressure = 101325; %[Pa]
%         airSpecificGasConst = 287.058; %[J kg^-1 K^-1]
%         airDensity = airPressure/(airSpecificGasConst*airTemperature); %[kg m^-3]
        airDensity = 1.2930; %[kg m^-3]
    end
    
    methods
        function obj = untitled(inputArg1,inputArg2)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

