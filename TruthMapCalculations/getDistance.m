function distance = getDistance(lat1, lon1, lat2, lon2)
%getDistance
%   Converts two coordinates into the distance apart in meters.
    R = 6378.137; 
    dLat = lat2 * pi / 180 - lat1 * pi / 180; %the pi/180 turns it into rad
    dLon = lon2 * pi / 180 - lon1 * pi / 180;
    a = sin(dLat/2) * sin(dLat/2) + cos(lat1 * pi / 180) * cos(lat2 * pi / 180) * sin(dLon/2) * sin(dLon/2);
    c = 2 * atan2(sqrt(a), sqrt(1-a));
    d = R * c;
    distance = d * 1000; % meters
end