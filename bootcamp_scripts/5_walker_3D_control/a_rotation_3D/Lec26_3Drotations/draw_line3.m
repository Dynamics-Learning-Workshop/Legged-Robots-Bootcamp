function draw_line3(p1, p2, varargin)
% help
% draw_line3 draw directional vector points in 3D with directional arrows
%
% draw_line3(p1, p2) draw line with defulat optional parmeters;
% draw_line3(p1, p2 , param1, val1, ...)
% The line with stretching(<-->) and compressive(>---<) arrows or without
% arrows (----) can be plotted.
%
% Input arguments:
% ----------------------
% p1, p2  : Cordinates of the two points in 3D
%
% Optional parameters ( passed as parameter/value pairs):
%--------------------------------------------------------
%
% 'LineColor'       :   Colour of line ( vector of colour or char)
%                       default : % 'b'
% 'LineWidth'       :   Width of the line (Scaler values)
%                       default :  2)
% 'ArrowDirection'  :   (0,1,2) (0 : p1--- p2, 1: p1 --> p2 2: and P1 <-> p2)
%                       default : 1
% 'ArrowLength'     :   Length of the arrow ( scaler values)
%                       default : 5
% 'ArrowIntend'     :   Intended arrow ( scaler value)
%                       default : 2
% 'ArrowAngle'      :   Angle of arrow with the main line in degree
%                       default : 45
% 'ArrowColor'      :   Arrow face clour ( in color vector or char)
%                       default : 'b'
% 'EdgeColor        :   arrow edge colour ( in colour vector or char or
%                       default : 'g'
% 'FaceAlpha        :   alpha values for face transparency of the arrow and line
%                       default : 1.0
% 'EdgeAlpha        :   alpha values for edge transparency of the arrow and line
%                       default : 1.0
% 'Linestyle        :   Linetyle on the linw and arrow face ('-' , '--',..)
%                       default : 'None'
%
% Example:
%
%   draw_line3([0 0 0]', [100 100 100]',...
%                     'Linecolor', 'b',...
%                     'LineWidth', 5 ,...
%                     'ArrowDirection', 2,...
%                     'ArrowLength', 10,....
%                     'ArrowIntend', 3,...
%                     'ArrowAngle', 45,...
%                     'ArrowColor', 'r')
%                
%  will draw the line with the specified optional parameters in the current plot               
%
% -------------------------------------------------------------------------
% Copyright 2012 Sabesan Sivapalan  
% Last edited: 25/09/2012           
% email: sabeshuom@gmail.com        
% -------------------------------------------------------------------------

%% check the given points are column format
%clc;
% p1 = [0 0 0]';
% p2=  [100 100 100]';
if ~exist('p1', 'var')  || ~exist('p2', 'var')
    disp('No input points.')
    return
end
if length(p1) ~=3  || length(p1) ~=3
    disp('Not proper 3D cordinates given for the input points.')
    return
end


% check whether new plot or plotting on the existing
hold_status = ishold;
if ~hold_status
    subplot(1,1,1,'replace');
    hold on;
end

if size(p1 , 1) == 1
    p1 = p1';
end

if size(p2 , 1) == 1
    p2 = p2';
end

% Optional properties
properties = {  'LineColor',...
                'LineWidth',...
                'ArrowDirection',...
                'ArrowLength',...
                'ArrowIntend',...
                'ArrowAngle',...
                'ArrowColor',...
                'EdgeColor',...
                'FaceAlpha',...
                'EdgeAlpha',...
                'LineStyle'};

% default values
values.LineColor        = 'b';
values.LineWidth        = 2;
values.ArrowDirection   = 2;
values.ArrowLength      = 5;
values.ArrowIntend      = 2;
values.ArrowAngle       = 45;
values.ArrowColor       = 'r';
values.EdgeColor        = 'g';
values.FaceAlpha        = 1.0;
values.EdgeAlpha        = 1.0;
values.LineStyle        = 'none';

given_property = varargin(1:2:end);
propertyValue = varargin(2:2:end);

% update the porperty values with the given optional parameters
for i=1:size(given_property, 2)
    validInput = sum(cell2mat(strfind(properties, given_property{i})));
    
    if validInput
        values.(sprintf('%s',given_property{i})) = propertyValue{i};
    end
    
end

% get the node parameters for the end point
node1 = get_nodes(p1, p2, values);

% check the arrow directions and plot the line and thte arrows according to that
% if angle of ArrowAngle > 90 then the arrow is drawn in a compressive way
%
if values.ArrowDirection   == 2;
    node2 = get_nodes(p2, p1, values);
    if(values.ArrowAngle > 0) && (values.ArrowAngle < 90);
        plot_line(node1{4}{1}, node2{4}{1}, node1{2}{2}, values);
    else
        plot_line(node1{2}{1}, node2{2}{1}, node1{2}{2}, values);
    end
        
    plot_arrow(node1{2}{1}, node1{3}{1}, node1{4}{1}, node1{2}{2}, values);
    plot_arrow(node2{2}{1}, node2{3}{1}, node2{4}{1}, node2{2}{2}, values);
else
    if(values.ArrowAngle > 0) && (values.ArrowAngle < 90);
        plot_line(node1{1}{1}, node1{4}{1}, node1{2}{2}, values);
    else
        plot_line(node1{1}{1}, node1{2}{1}, node1{2}{2}, values);
    end
    if values.ArrowDirection   == 1;
        plot_arrow(node1{2}{1}, node1{3}{1}, node1{4}{1}, node1{2}{2}, values);
    end
end 

if ~hold_status
    hold off;
end
axis equal;
% plotting stuff

end

function node = get_nodes(p1, p2, values)
% Calculation of angles
p = p2- p1;
rz = atan2(p(2), p(1));
ry = asin(p(3)/sqrt(sum(p.^2)));
node{1}= {p1, [1 0 0;0 1 0; 0 0 1]'};
% node2
cmd = ['x' 'y' 'z' 'c' 'b' ];
len_x = sqrt(sum((p1 - p2).^2));
para = [len_x 0 0  rz ry];
node{2} = gen_node(node{1},cmd,para);
if sum(node{2}{1} == p2) ~=3
    ry =  -asin(p(3)/sqrt(sum(p.^2)));
    para = [len_x 0 0  rz ry];
    node{2} = gen_node(node{1},cmd,para);
end
%node3
cmd = 'x';
if (values.ArrowAngle > 0) && (values.ArrowAngle < 90)
para = -values.ArrowLength;
else
para = values.ArrowLength;    
end
node{3} = gen_node(node{2}, cmd, para);

% node4
cmd = 'x';
para = sign(para) * (values.ArrowLength - values.ArrowIntend);
node{4} = gen_node(node{2}, cmd, para);
end

function plot_arrow(node1, node2, node3, rot, values)
    % arrow cordinates
    if (values.ArrowAngle > 0) && (values.ArrowAngle < 90)
        arrow_width1 = abs(values.ArrowLength * tan(pi * values.ArrowAngle/180));
        arrow_width2 = 0;
    else
        arrow_width2 = abs(values.ArrowLength * tan(pi * (values.ArrowAngle - 90)/180));
        arrow_width1 = 0;
    end
        
    seg = [1, values.ArrowLength/2 0 0, 0 arrow_width2 arrow_width2, 0 arrow_width1 arrow_width1];
    [s1_x s1_y s1_z] = truncated_cone_sk(seg, (node1 + node2)./2, rot);
    seg = [1, values.ArrowIntend/2 0 0, 0 arrow_width2 arrow_width2, 0 arrow_width1 arrow_width1];
    [s2_x s2_y s2_z] = truncated_cone_sk(seg, (node2 + node3)./2, rot);

    s1 = surf(s1_x, s1_y, s1_z);
    set(s1, 'FaceColor', values.ArrowColor , 'EdgeColor', values.EdgeColor, 'EdgeAlpha', values.EdgeAlpha, 'FaceAlpha', values.FaceAlpha, 'LineStyle', values.LineStyle);
    s2 = surf(s2_x, s2_y, s2_z);
    set(s2, 'FaceColor', values.ArrowColor , 'EdgeColor', values.EdgeColor, 'EdgeAlpha', values.EdgeAlpha, 'FaceAlpha', values.FaceAlpha, 'LineStyle',values.LineStyle);
 end

function plot_line(node1, node2, rot, values)
len = abs(sqrt(sum((node2- node1).^2)));
seg = [1, len/2, 0 0 , 0 values.LineWidth values.LineWidth, 0 values.LineWidth values.LineWidth];
[s1_x s1_y s1_z] = truncated_cone_sk(seg, (node1+ node2)./2, rot);

seg = [1, 0 0 0, 0 0 0, 0 values.LineWidth values.LineWidth];
[s2_x s2_y s2_z] = truncated_cone_sk(seg, node1, rot);

seg = [1, 0 0 0, 0 0 0, 0 values.LineWidth values.LineWidth];
[s3_x s3_y s3_z] = truncated_cone_sk(seg, node2, rot);

s1 = surf(s1_x, s1_y, s1_z);
set(s1, 'FaceColor', values.LineColor , 'EdgeColor', values.EdgeColor, 'EdgeAlpha', values.EdgeAlpha, 'FaceAlpha',values.FaceAlpha, 'LineStyle', values.LineStyle);
s2 = surf(s2_x, s2_y, s2_z);
set(s2, 'FaceColor', values.LineColor , 'EdgeColor', values.EdgeColor, 'EdgeAlpha', values.EdgeAlpha, 'FaceAlpha',values.FaceAlpha, 'LineStyle', values.LineStyle);
s3 = surf(s3_x, s3_y, s3_z);
set(s3, 'FaceColor', values.LineColor , 'EdgeColor', values.EdgeColor, 'EdgeAlpha', values.EdgeAlpha, 'FaceAlpha', values.FaceAlpha, 'LineStyle', values.LineStyle);
end

function node = gen_node(node_prev,cmd,para)
n = node_prev{1};
r = node_prev{2};
for i=1:length(cmd)
    if char(cmd(i)) == 'a' || char(cmd(i)) == 'b' || char(cmd(i)) == 'c' || char(cmd(i)) == 'd' || char(cmd(i)) == 'e' || char(cmd(i)) == 'f'
        switch (char(cmd(i)))
            case 'a'
                r = rotate_x(r,para(i));
                
            case 'b'
                r = rotate_y(r,para(i));
                
            case 'c'
                r = rotate_z(r,para(i));
                
            case 'd'
                r = rotate_x(r,-para(i));
                
            case 'e'
                r = rotate_y(r,-para(i));
                
            case 'f'
                r = rotate_z(r,-para(i));
        end
    end
end

for i=1:length(cmd)
    if char(cmd(i)) == 'x' || char(cmd(i)) == 'y' || char(cmd(i)) == 'z' || char(cmd(i)) == 'i' || char(cmd(i)) == 'j' || char(cmd(i)) == 'k'
        switch (char(cmd(i)))
            
            case 'x'
                n = translate(n,r,[para(i),0 0]');
            case 'y'
                n = translate(n,r,[ 0 para(i),0]');
            case 'z'
                n = translate(n,r,[0 0 para(i)]');
            case 'i'
                n = translate(n,r,[-para(i),0 0]');
            case 'j'
                n = translate(n,r,[ 0 -para(i),0]');
            case 'k'
                n = translate(n,r,[0 0 -para(i)]');
        end
    end
    
end

node = {n, r};
end

function [x y z] = truncated_cone_sk(seg, c, rot)
% Example:
% seg = [1 20 0 0 0 10 20 0 1 10];
% Input: seg
% lbl  centre_x  centre_y  centre_z start_x start_y start_z end_x end_y end_z;
n = 40;
m = 2;
theta = (0:n)/n*2*pi;
sintheta = sin(theta); sintheta(n+1) = 0;

[val ind] = max([seg(2),seg(3),seg(4)]) ;
switch ind
    case 1
        ry = [seg(9); seg(6)];
        rz = [seg(10); seg(7)];
        x = [-seg(2) seg(2)]' * ones(1,n+1);
        y = ry * cos(theta);
        z = rz * sintheta;
    case 2
        rx = [seg(8); seg(5)];
        rz = [seg(10); seg(7)];
        y = [-seg(3) seg(3)]' * ones(1,n+1);
        x = rx * cos(theta);
        z = rz * sintheta;
    case 3
        rx = [seg(8); seg(5)];
        ry = [seg(9); seg(6)];
        z = [-seg(4) seg(4)]' * ones(1,n+1);
        x = rx * cos(theta);
        y = ry * sintheta;
    
end

q = [x(:) y(:) z(:)]';
c_r = repmat(c, [1,size(q,2)]);
q = c_r + rot * (q);
x = reshape(q(1,:),size(x));
y = reshape(q(2,:),size(y));
z = reshape(q(3,:),size(z));
end

function r = rotate_x(r_prev , x)
r = [     1       0       0 ;       0   cos(x) -sin(x);      0    sin(x) cos(x)];
r = r_prev * r;
end

function r = rotate_y(r_prev , x)
r = [ cos(x)      0   sin(x);       0       1       0 ; -sin(x)       0  cos(x)];
r = r_prev * r;
end

function r = rotate_z(r_prev , x)
r = [ cos(x) -sin(x)      0 ;   sin(x)  cos(x)      0 ;       0       0      1 ];
r = r_prev * r;
end

function n = translate(n,r,x)
n = n + r * x;
end



