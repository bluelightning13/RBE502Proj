
	
	% function to calculate thrust
	function thf = thrust(cInput, k)
		thf = [0; 0; k * sum(cInput)];
	end
