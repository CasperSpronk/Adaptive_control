function [ rate_offset, inverted ] = get_coordination_rate ( bank_angle, meas_pitch_angle, aspeed, p)
%UNTITLED Summary of this function goes here
%   All variables expressed in degrees!

 roll_ff = p(1);
 gravity = p(2);
 EAS2TAS = p(3);
 airspeed_min = p(4);

if bank_angle < 90
    bank_angle = constrain_value(bank_angle, -80, 80);
    inverted = false;
else
    inverted = true;
    if bank_angle > 0
      bank_angle = constrain_value(bank_angle, 100, 180);  
    else
       bank_angle = constrain_value(bank_angle, -180, 100);
    end
end

if abs(meas_pitch_angle) > 70
    rate_offset = 0;
else
    rate_offset = cosd(meas_pitch_angle)...
        *abs((gravity / max((aspeed * EAS2TAS) , airspeed_min)) * tand(bank_angle) * sind(bank_angle))...
        * roll_ff;
end

if inverted
    rate_offset = -rate_offset;
end


end

