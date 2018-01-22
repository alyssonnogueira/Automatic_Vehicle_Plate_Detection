%Converter para  imagem binaria
I = imread('placa2.png');
ImBinary = rgb2gray(I);

figure
subplot(1,5,1)
imshow(ImBinary)
hold on
title('Imagem em Tons de cinza')
hold off

se = strel('rectangle',[50 80]);
mexicanImage= imtophat(ImBinary, se);

%Show remaining regions
%figure
subplot(1,5,2)
imshow(mexicanImage)
hold on
title('Filtro Chapeu Mexicano')
hold off

%feature detection and extraction
[mserRegions, mserConnComp] = detectMSERFeatures(mexicanImage, ...
    'RegionAreaRange',[200 8000],'ThresholdDelta',9);

subplot(1,5,3)
imshow(mexicanImage)
hold on
plot(mserRegions, 'showPixelList', true,'showEllipses',false)
title('Destacando possiveis caracteres')
hold off

% Use regionprops to measure MSER properties
mserStats = regionprops(mserConnComp, 'BoundingBox', 'Eccentricity', ...
    'Solidity', 'Extent', 'Euler', 'Image');

% Get bounding boxes for all the regions
bboxes = vertcat(mserStats.BoundingBox);

% Convert from the [x y width height] bounding box format to the [xmin ymin
% xmax ymax] format for convenience.
xmin = bboxes(:,1);
ymin = bboxes(:,2);
xmax = xmin + bboxes(:,3) - 1;
ymax = ymin + bboxes(:,4) - 1;

% Expand the bounding boxes by a small amount.
expansionAmount = 0.02;
xmin = (1-expansionAmount) * xmin;
ymin = (1-expansionAmount) * ymin;
xmax = (1+expansionAmount) * xmax;
ymax = (1+expansionAmount) * ymax;

% Clip the bounding boxes to be within the image bounds
xmin = max(xmin, 1);
ymin = max(ymin, 1);
xmax = min(xmax, size(mexicanImage,2));
ymax = min(ymax, size(mexicanImage,1));

% Show the expanded bounding boxes

expandedBBoxes = [xmin ymin xmax-xmin+1 ymax-ymin+1];
IExpandedBBoxes = insertShape(mexicanImage,'Rectangle',expandedBBoxes,'LineWidth',3);

%figure
subplot(1,5,4)
imshow(IExpandedBBoxes)
title('Aplicando boundary box')

% Compute the overlap ratio
overlapRatio = bboxOverlapRatio(expandedBBoxes, expandedBBoxes);

% Set the overlap ratio between a bounding box and itself to zero to
% simplify the graph representation.
n = size(overlapRatio,1);
overlapRatio(1:n+1:n^2) = 0;

% Create the graph
g = graph(overlapRatio);

% Find the connected text regions within the graph
componentIndices = conncomp(g);

% Merge the boxes based on the minimum and maximum dimensions.
xmin = accumarray(componentIndices', xmin, [], @min);
ymin = accumarray(componentIndices', ymin, [], @min);
xmax = accumarray(componentIndices', xmax, [], @max);
ymax = accumarray(componentIndices', ymax, [], @max);

% Compose the merged bounding boxes using the [x y width height] format.
textBBoxes = [xmin ymin xmax-xmin+1 ymax-ymin+1];

% Remove bounding boxes that only contain one text region
numRegionsInGroup = histcounts(componentIndices);
textBBoxes(numRegionsInGroup == 1, :) = [];

% Show the final text detection result.
ITextRegion = insertShape(mexicanImage, 'Rectangle', textBBoxes,'LineWidth',3);

%figure
subplot(1,5,5)
imshow(ITextRegion)
ocrtxt = ocr(ImBinary, textBBoxes);
value = [ocrtxt.Text];
title([ocrtxt.Text])
