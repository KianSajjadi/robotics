classdef JointsTools < handle
    properties
    end

    methods(Static)
		function jointVelocities = getJointVelocities(jacobianMatrix, endEffectorVelocity, weightedMatrix)
			jointVelocities = inv(weightedMatrix) * transpose(jacobianMatrix) * (inv(jacobianMatrix * inv(weightedMatrix) * transpose(jacobianMatrix))) * endEffectorVelocity;
		end
		
		function weightedMatrix = getWeightedMatrix(q, qMax, qMin, qVelocities, c)
			weights = ones(1, 7);
			for i = 1:7
				hOfQ = getHOfQ(q(i), qMax(i), qMin(i), c(i));
				midPoint = getMidPoint(qMax(i), qMin(i));
				weight = get_weight(q(i), hOfQ, qVelocities(i), midPoint);
				weights(1, i) = weight;
			end
			weightedMatrix = diag(weights);
		end
	end
end

function hOfQ = getHOfQ(q, qMax, qMin, c)
	hOfQ = ((qMax - qMin) * (2*q - qMax - qMin)/(c*(qMax - q).^2)*((q-qMin).^2));
end
		
function midPoint = getMidPoint(qMax, qMin)
    midPoint = (qMax+qMin)/2;
end
		
function weight = get_weight(q, hOfQ, q_velocity, midPoint)
    if q > midPoint && q_velocity > 0 || q < midPoint && q_velocity < 0
		weight = 1 + abs(hOfQ);
	else
		weight = 1;
	end
end