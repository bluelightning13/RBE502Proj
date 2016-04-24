% main function where everything runs
function outline()
    %main function 
    points=[0 5 0 ; 0 2 0 ; 0 1 0 ];
    cubicPoly = (cscvn(points))
    fnplt(cubicPoly)
    coAos = cubicPoly.coefs  %this is a 4*6 array (with 3pts) with the coeffs and offsets
    breaks = cubicPoly.breaks %this is the breaks in the graphs (array of 3)
    
    
    
end
