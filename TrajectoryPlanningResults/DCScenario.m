% Setup DC package delivery scenario 
GWU = [38.900529, -77.049058];
sichuanPavilion = [38.902443, -77.042052];
watergateHotel = [38.899612, -77.055408];
initialPositions = [GWU; sichuanPavilion; watergateHotel];
cornerLat = [38.9056   38.9056   38.8972   38.8972 ];
cornerLon = [-77.0398  -77.0614  -77.0614  -77.0398];

traderJos = [38.904193, -77.053129]; %GWU
tonicAtQuincy = [38.898055, -77.046471]; %GWU
foundingFarmers = [38.90028, -77.044415]; %sichuanPavilion
theRitzHotel = [38.904526, -77.049072]; %sichuanPavilion 
fourSeason = [38.904269, -77.057294]; %watergateHotel
bakedAndWired = [38.903887, -77.060437]; %watergateHotel
finalPositions = [traderJos;foundingFarmers;tonicAtQuincy;theRitzHotel;fourSeason;bakedAndWired];
%% Scenario 2 
clear all; clc
Maydan = [38.920027, -77.03127];
watergateHotel = [38.899612, -77.055408];
JoesStake = [38.900027, -77.033922];
initialPositions = [Maydan; JoesStake; watergateHotel];
cornerLat = [ 38.9228   38.9228   38.8990   38.8990];
cornerLon = [-77.0274  -77.0580  -77.0580  -77.0274];

PhilipsCollection = [38.91151, -77.046854];%Maydan
wholeFoolds = [38.909499, -77.033568];%Maydan
Dumbarton = [38.910883, -77.055618];%watergateHotel
foundingFarmers = [38.90028, -77.044415]; %watergateHotel
agoralDC = [38.91063, -77.038251];%JoesStake
% MuseumMansion = [38.908363, -77.045874];%JoesStake
AperoLa = [38.909246, -77.055494];%JoesStake
finalPositions = [PhilipsCollection;wholeFoolds;Dumbarton;foundingFarmers;agoralDC;AperoLa];
Dupont = [38.910944, -77.042726, 10];

[initialXY, finalXY] = geoToCartesian(initialPositions, finalPositions, Dupont);
%% DFW
latEnd = [32.864085567567571,32.862029441441443,32.863482234234233,32.852869441441442,...
32.863255657657660,32.861800072072072];
longEnd = [ -96.806916269255453
 -96.794711460015591
 -96.815022884748288
 -96.804653628708820
 -96.799023948217126
 -96.814951882281392];
finalPositions = [latEnd',longEnd];
latStart = [  32.872522864864862
  32.852581243243243
  32.852164936936937];
longStart = [-96.792586105205572
 -96.795005230041056
 -96.814858785391237];
initialPositions = [latStart,longStart];
createPackageDeliveryKML(initialPositions, finalPositions, 'package_delivery_scenario.kml');

% Assume initialXY and finalXY are already computed
% distances = computePairwiseDistances(initialXY, finalXY);


function distances = computePairwiseDistances(initialXY, finalXY)
    % Subtract every point in finalXY from every point in initialXY
    % This creates two matrices, one for x-coordinate differences and one for y-coordinate differences
    diffX = finalXY(:,1)' - initialXY(:,1);
    diffY = finalXY(:,2)' - initialXY(:,2);

    % Compute the square of the differences
    squaredDiffs = diffX.^2 + diffY.^2;

    % Calculate the Euclidean distance
    distances = sqrt(squaredDiffs);
end
