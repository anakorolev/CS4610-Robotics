% TODO: You write this function!
% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%                     effector position to reach <position>
%                     (orientation is to be ignored)
function q = Q2(f,qInit,posGoal)

    jacob = f.jacob0(qInit)
    jacobV = jacob(1:3,:)
    x = f.fkine(qInit)
    xDes=[1,0,0,posGoal(1);0,1,0,posGoal(2);0,0,1,posGoal(3);0,0,0,1]
    subtract = [x.n(1),x.o(1),x.a(1),x.t(1);x.n(2),x.o(2),x.a(2),x.t(2);x.n(3),x.o(3),x.a(3),x.t(3);0,0,0,1;]
    deltaXDes = 0.1.*(xDes - subtract)
    PIJacobV = pinv(jacobV)
    deltaQDes = pinv(jacobV).*deltaXDes
    q = qInit + deltaQDes

end


