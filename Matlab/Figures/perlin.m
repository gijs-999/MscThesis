function s = perlin (m,w)
  s = zeros([m,1]);     % Prepare output image (size: m x m)
  i = 0;
  for i_w = 1:w
    i = i + 1;
    d = interp2(randn([m,2]), i-1, 'spline');
    s = s + i * d(1:m, 1);
  end
  s = (s - min(min(s(:,:)))) ./ (max(max(s(:,:))) - min(min(s(:,:))));
end