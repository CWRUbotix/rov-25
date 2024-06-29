rm surface_transceiver/rov_common.cpp surface_transceiver/rov_common.hpp float_transceiver/rov_common.cpp float_transceiver/rov_common.hpp

cd surface_transceiver
ln -s ../include/rov_common.hpp rov_common.hpp
ln -s ../src/rov_common.cpp rov_common.cpp

cd ../float_transceiver
ln -s ../include/rov_common.hpp rov_common.hpp
ln -s ../src/rov_common.cpp rov_common.cpp

cd ..