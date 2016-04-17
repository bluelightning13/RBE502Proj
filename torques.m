% compute the torques
	function tau = torques()
		tau = [L*k*(cInput(1) - cInput(3))  L*k*(cInput(2) - cInput(4))  b*(cInput(1) - cInput(2) + cInput(3) - cInput(4)) ];
	end