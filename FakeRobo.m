classdef FakeRobo
    properties
        s
        n % number of joints
    end
    
    methods
        function obj = FakeRobo(s, n)
            obj.s = s;
            obj.n = n;
        end
        
        function qdd = accel(obj, q, qd, torque)
            % returns a column vector of joint accelerations
            % inputs are row vectors
            qdd = obj.Dmat(q)\(torque' - obj.Cmat(q)*qd' + obj.Gmat(q));
        end
        
        function torque = rne(obj, q, qd, qdd)
            % returns column vector of torques
            torque = obj.Dmat(q)*qdd' + obj.Cmat(q)*qd' + obj.Gmat(q);
        end
        
        function D = Dmat(obj, q)
            D = zeros(3);
            D(1,1) = -3*sin(q(1)) * obj.s(1);
            D(1,2) = 0;
            D(1,3) = -2*cos(q(2)+q(3)) * obj.s(2);
            
            D(2,1) = 4.5*sin(q(2))*sin(q(3)) * obj.s(3) + cos(q(1)) * obj.s(1);
            D(2,2) = pi * obj.s(4);
            D(2,3) = cos(pi + q(2)) * obj.s(2) * obj.s(3);
            
            D(3,1) = 1.5 * sin(q(3))*cos(q(1))* obj.s(5);
            D(3,2) = -3;
            D(3,3) = 0;
        end
        
        function C = Cmat(obj, q)
            C = zeros(3);
%             C(1,1) = 0;
%             C(1,2) = -3*sin(q(3) - pi/2) * obj.s(3);
%             C(1,3) = 5 - 2.6*sin(q(2));
%             
%             C(2,1) = 0;
%             C(2,2) = 1*cos(q(1))*cos(q(2)) * obj.s(4) + 2*sin(q(1)) * obj.s(3);
%             C(2,3) = 5*cos(q(3)) * obj.s(2);
%             
%             C(3,1) = -2 * obj.s(1) + 1*sin(q(2) + q(3)) * obj.s(5);
%             C(3,2) = 0;
%             C(3,3) = 3 - 1.5 * obj.s(4);
        end
        
        function G = Gmat(obj, q)
            % returns column vector of torques
            G = zeros(3, 1);
%             G(1) = 0;
%             G(2) = -3.5*sin(q(1)) * obj.s(2);
%             G(3) = -0.8*cos(q(3) - q(2)) * obj.s(5) + 4*sin(q(2)) * obj.s(3);
        end
    end
end