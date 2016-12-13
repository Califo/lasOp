//#include "stdafx.h"

#include "LasOperator.h"
#include <iostream>
#include <math.h> 

using namespace std;

/*===================================================================
Constructor of CLasOperator
Initialization.
===================================================================*/

 CLasOperator:: CLasOperator(){
	 isLas = false;
	 avail = false;
}

/*===================================================================
Destructor of CLasOperator
Free space.
===================================================================*/

 CLasOperator::~ CLasOperator(){

}

/*===================================================================
CLasOperator::readLasFile
1. Open an LAS file specified by the parameter file_name.
2. Read the data block corresponding to public header block and store it.
3. Read the data block corresponding to variable length record headers and store them.
4. Read the point data record and store them.
===================================================================*/

bool  CLasOperator::readLasFile(const char *file_name){

	clearData();
	filename.assign(file_name);

	// Open an Las File
	fstream file;
	file.open(file_name, ios::in | ios::binary);
	if(file.fail()) {
		file.close();
		return false;
	}

	// Read public header block
	if(!readPulicHeaderBlock(file)) {
		file.close();
		return false;
	}

	// Read variable length record headers
	if(!readVariableLengthRecords(file)) {
		file.close();
		return false;		
	}
	
	// Read point data records
	if(!readPointDataRecords(file)){
		file.close();
		return false;
	}

	file.close();
	isLas = true;
	avail = true;

	return true;
}

/*===================================================================
CLasOperator::readXYZFile
1. Open an XYZ file specified by the parameter file_name.
2. Read the data points with x,y,z coordinations.
3. Store the coordinations into point3d.
===================================================================*/

bool  CLasOperator::readXYZFile(const char *file_name){

	clearData();
	filename.assign(file_name);

	// Open an XYZ file
	fstream file;
	file.open(file_name,ios::in);
	if(file.fail()){
		file.close();
		return false;
	}

	Point3D point;
	// Read data points
	while(!file.eof()){
		file>>point.x;
		file>>point.y;
		file>>point.z;
		point3d.push_back(point);
	}
	point3d.pop_back();
	file.close();
	isLas = false;
	avail = true;

	return true;
}

/*===================================================================
CLasOperator::exportData

===================================================================*/

bool  CLasOperator::exportData(const char *file_name,
        bool RGB,                                 // RBG value
		bool intensity,                           // Intensity value
		bool returnNum,                           // Return number
		bool numOfReturns,                        // Number of returns (given pulse)
		bool scandir,                             // Scan direction
		bool flightline,                          // Edge of the flight line
		bool classification,                      // Classification
		bool GPS){                                // GPS time

	// Open a file
	fstream file;
	file.open(file_name,ios::out);
	if(file.fail()){
		file.close();
		return false;
	}

	for(size_t i=0;i<pHeader.pointRecordNum;++i){

		file<<fixed<<point3d[i].x<<"\t"<<point3d[i].y<<"\t"<<point3d[i].z;

		if(RGB)
			file<<fixed<<"\t"<<pointData[i].red<<"\t"<<pointData[i].green<<"\t"<<pointData[i].blue;
		if(intensity)
			file<<fixed<<"\t"<<pointData[i].intensity;
		if(returnNum)
			file<<fixed<<"\t"<<(pointData[i].mask & 0x07);
		if(numOfReturns)
			file<<fixed<<"\t"<<((pointData[i].mask & 0x38)>>3);
		if(scandir)
			file<<fixed<<"\t"<<((pointData[i].mask & 0x40)>>6);
		if(flightline)
			file<<fixed<<"\t"<<((pointData[i].mask & 0x80)>>7);
		if(classification)
			file<<fixed<<"\t"<<(short)pointData[i].classification;
		if(GPS)
			file<<fixed<<"\t"<<pointData[i].GPS;
		file<<endl;
	}

	file.close();
	return true;
}

/*===================================================================
CLasOperator::lasSampling
Sample the point cloud data specified by parameter inter which is the
sampling interval and store into a las file.
===================================================================*/

bool CLasOperator::lasSampling(const char *file_name, unsigned long inter){

	// Open a file
	fstream file;
	file.open(file_name,ios::out|ios::binary);
	if(file.fail()){
		file.close();
		return false;
	}

	PublicHeaderBlock ph;

	memcpy(&ph,&pHeader,sizeof(PublicHeaderBlock));
	ph.pointRecordNum = pHeader.pointRecordNum/inter; // Set the sampled point record number
	ph.maxX = pointData[0].x;
	ph.minX = pointData[0].x;
	ph.maxY = pointData[0].y;
	ph.minY = pointData[0].y;
	ph.maxZ = pointData[0].z;
	ph.minZ = pointData[0].z;
	for(size_t i=inter;i<pointData.size();i+=inter){ // Compute the min and max of x,y and z
		if(ph.maxX<pointData[i].x)
			ph.maxX = pointData[i].x;
		if(ph.minX>pointData[i].x)
			ph.minX = pointData[i].x;
		if(ph.maxY<pointData[i].y)
			ph.maxY = pointData[i].y;
		if(ph.minY>pointData[i].y)
			ph.minY = pointData[i].y;
		if(ph.maxZ<pointData[i].z)
			ph.maxZ = pointData[i].z;
		if(ph.minZ>pointData[i].z)
			ph.minZ = pointData[i].z;
	}
	// Compute the offsetToData
	ph.offsetToData = ph.headerSize+varHeader.size()*sizeof(VariableLengthRecordHeader);
	// Write the public header block into file
	file.write((const char *)&ph,sizeof(PublicHeaderBlock));
	// Write the variable length headers into file
	for(size_t i=0;i<varHeader.size();++i){
		file.write((const char *)&varHeader[i],sizeof(VariableLengthRecordHeader));
	}
	// Write the data point record into file
	for(size_t i=0;i<pointData.size();i+=inter){
		file.write((const char *)&pointData[i],20);
		//Distinguish different point data record format
		switch(pHeader.pointRecordLen){
		case 28:
			file.write((const char *)&pointData[i].GPS,sizeof(double));
			break;
		case 26:
			file.write((const char *)&pointData[i].red, 3*sizeof(unsigned short));
			break;
		case 34:
			file.write((const char *)&pointData[i].GPS, sizeof(double)+3*sizeof(unsigned short));
			break;
		}
	}
	file.close();
	return true;
}

/*===================================================================
CLasOperator::xyzSampling
Sample the point cloud data specified by parameter inter which is the
sampling interval and store into a xyz file.
===================================================================*/

bool CLasOperator::xyzSampling(const char *file_name, unsigned long inter){
	// Open a file
	fstream file;
	file.open(file_name,ios::out);
	if(file.fail()){
		file.close();
		return false;
	}
	for(size_t i=0;i<point3d.size();i+=inter){
		file<<fixed<<point3d[i].x<<"\t"<<point3d[i].y<<"\t"<<point3d[i].z<<endl;
	}
	file.close();
	return true;
}

/*===================================================================
CLasOperator::pointNum
Return the number of the data points.
===================================================================*/

unsigned long CLasOperator::pointNum(){ 
	return point3d.size();
}

std::vector<Point3DC>& CLasOperator::getPoints(){
	return points;
}
/*===================================================================
CLasOperator::getPointData
Return the vector containing the data point coordinations.
===================================================================*/

std::vector<Point3D>& CLasOperator::getPointData(){                
	return point3d;
}

/*===================================================================
CLasOperator::getPointRecords
Return the vector containing the data point records.
===================================================================*/

std::vector<PointDataRecord>& CLasOperator::getPointRecords(){  
	return pointData;
}

/*===================================================================
CLasOperator::getPublicHeader
Return the structure of the public header block.
===================================================================*/

PublicHeaderBlock& CLasOperator::getPublicHeader(){
	return pHeader;
}

/*===================================================================
CLasOperator::isLasFile
Return a boolean value indicating whether the imported file is an 
las file or an XYZ file.
===================================================================*/

bool CLasOperator::isLasFile(){
	return isLas;
}

/*===================================================================
CLasOperator::available
Return a boolean value indicating whether an las file or an XYZ file
has been imported into the structure.
===================================================================*/

bool CLasOperator::available(){
	return avail;
}

/*===================================================================
CLasOperator::clearData
Clear all the information about the imported file.
===================================================================*/

void CLasOperator::clearData(){
	varHeader.clear();
	pointData.clear();
	point3d.clear();
	isLas = false;
	avail = false;
}

/*===================================================================
CLasOperator::readPublicHeaderBlock
1. Read the data block corresponding to public header block.
2. Store the information into pHeader
3. Check the validity of the LAS file
===================================================================*/

bool  CLasOperator::readPulicHeaderBlock(fstream &file){

	char *pHeaderBuf = new char[sizeof(PublicHeaderBlock)];
	file.read(pHeaderBuf, sizeof(PublicHeaderBlock));
	memcpy(&pHeader, pHeaderBuf,sizeof(PublicHeaderBlock));
	delete [] pHeaderBuf;
	//if(strcmp(pHeader.fileSign, "LASF") != 0) {
		//return false;
	//}
	return true;
}

/*===================================================================
CLasOperator::readVariableLengthRecords
1. Read the data block corresponding to variable length record headers iteratively
2. Store them into varHeader
===================================================================*/

bool  CLasOperator::readVariableLengthRecords(fstream &file){

	size_t varHeaderSize = sizeof(VariableLengthRecordHeader);
	char *varHeaderBuf = new char[varHeaderSize];
	VariableLengthRecordHeader pvarHeader;
	unsigned long startPoint = pHeader.headerSize;

	for(unsigned long i=0;i<pHeader.varRecordNum;++i){
		file.seekg(startPoint);
		file.read(varHeaderBuf, varHeaderSize);
		memcpy(&pvarHeader, varHeaderBuf, varHeaderSize);
		startPoint += pvarHeader.recordLength + varHeaderSize;
		varHeader.push_back(pvarHeader);
	}

	delete [] varHeaderBuf;
	return true;
}

/*===================================================================
CLasOperator::readPointDataRecords

===================================================================*/

bool  CLasOperator::readPointDataRecords(fstream &file){

	file.seekg(pHeader.offsetToData);
	PointDataRecord pointRecord;
	Point3D point;
	Point3DC point2;
	char *pRecord = new char[pHeader.pointRecordLen];

	for (unsigned long i=0;i<pHeader.pointRecordNum;++i){
		file.read(pRecord,pHeader.pointRecordLen);
		memcpy(&pointRecord,pRecord,20);

		//Distinguish different point data record format
		switch(pHeader.pointRecordLen){
		case 28:
			memcpy(&pointRecord.GPS, &pRecord[20],sizeof(double));
			break;
		case 26:
			memcpy(&pointRecord.red, &pRecord[20],3*sizeof(unsigned short));
			break;
		case 34:
			memcpy(&pointRecord.GPS, &pRecord[20],sizeof(double)+3*sizeof(unsigned short));
			break;
		}
		point.x = pointRecord.x*pHeader.xScale+pHeader.xOffset;
		point.y = pointRecord.y*pHeader.yScale+pHeader.yOffset;
		point.z = pointRecord.z*pHeader.zScale+pHeader.zOffset;
		//std::cout << point.x << "   " << point.y << "   " << point.z << std::endl;
		point2.x = point.x;
		point2.y = point.y;
		point2.z = point.z;
		point2.red = pointRecord.red;
		point2.green = pointRecord.green;
		point2.blue = pointRecord.blue;
		points.push_back(point2);
		pointData.push_back(pointRecord);
		point3d.push_back(point);
		if(i % 10000000 == 0)
			std::cout << i << std::endl;

	}
	std::cout << "total: " << points.size() << std::endl;
	return true;
}
/*===================================================================
CLasOperator::saveLasFile
Save the given 3D points into a Las file.
===================================================================*/

bool CLasOperator::saveLasFile(const char *file_name,std::vector<Point3D> *points){

	// Open a las file
	std::fstream lasFile;
	lasFile.open(file_name,std::ios::out|std::ios::binary);
	if(lasFile.fail())
		return false;

	// Create PulicHeaderBlock
	PublicHeaderBlock publicHeader;
	sprintf(publicHeader.fileSign,"LASF");
	publicHeader.fileSourceID = 0;
	publicHeader.reserved = 0;
	publicHeader.GUID1 = 0;
	publicHeader.GUID2 = 0;
	publicHeader.GUID3 = 0;
	memset(publicHeader.GUID4,0,sizeof(publicHeader.GUID4));

	publicHeader.versionMajor = 1;
	publicHeader.versionMinor = 2;
	sprintf(publicHeader.systemID,"ddd");
	sprintf(publicHeader.GenSoft,"Allen Software");
	publicHeader.creationDay = 1;
	publicHeader.creationYear = 2013;
	publicHeader.headerSize = sizeof(PublicHeaderBlock);
	publicHeader.offsetToData = sizeof(PublicHeaderBlock);
	publicHeader.varRecordNum = 0;
	publicHeader.dataFormat = 3;
	publicHeader.pointRecordLen = sizeof(PointDataRecord);
	publicHeader.pointRecordNum = points->size();
	memset(publicHeader.returnPointNum,0,sizeof(publicHeader.returnPointNum));

	publicHeader.maxX = -DBL_MAX;
	publicHeader.minX = DBL_MAX;
	publicHeader.maxY = -DBL_MAX;
	publicHeader.minY = DBL_MAX;
	publicHeader.maxZ = -DBL_MAX;
	publicHeader.minZ = DBL_MAX;
	for(size_t i=0;i<points->size();++i){
		if((*points)[i].x>publicHeader.maxX)
			publicHeader.maxX = (*points)[i].x;
		if((*points)[i].x<publicHeader.minX)
			publicHeader.minX = (*points)[i].x;
		if((*points)[i].y>publicHeader.maxY)
			publicHeader.maxY = (*points)[i].y;
		if((*points)[i].y<publicHeader.minY)
			publicHeader.minY = (*points)[i].y;
		if((*points)[i].z>publicHeader.maxZ)
			publicHeader.maxZ = (*points)[i].z;
		if((*points)[i].z<publicHeader.minZ)
			publicHeader.minZ = (*points)[i].z;
	}

	double scaleX = publicHeader.maxX-publicHeader.minX>DBL_EPSILON?publicHeader.maxX-publicHeader.minX:DBL_EPSILON;
	double scaleY = publicHeader.maxY-publicHeader.minY>DBL_EPSILON?publicHeader.maxY-publicHeader.minY:DBL_EPSILON;
	double scaleZ = publicHeader.maxZ-publicHeader.minZ>DBL_EPSILON?publicHeader.maxZ-publicHeader.minZ:DBL_EPSILON;
	publicHeader.xScale = 1.0e-9 * scaleX;
	publicHeader.yScale = 1.0e-9 * scaleY;
	publicHeader.zScale = 1.0e-9 * scaleZ;
	publicHeader.xOffset = publicHeader.minX;
	publicHeader.yOffset = publicHeader.minY;
	publicHeader.zOffset = publicHeader.minZ;

	// Write pulbic header block
	lasFile.write((char *)(&publicHeader),sizeof(PublicHeaderBlock));

	// Export data points
	for(size_t i=0;i<points->size();++i){
		PointDataRecord pointRecord;
		pointRecord.x = static_cast<long>(((*points)[i].x-publicHeader.xOffset)/publicHeader.xScale);
		pointRecord.y = static_cast<long>(((*points)[i].y-publicHeader.yOffset)/publicHeader.yScale);
		pointRecord.z = static_cast<long>(((*points)[i].z-publicHeader.zOffset)/publicHeader.zScale);
		//pointRecord.intensity = static_cast<unsigned short>((*points)[i].intensity);
		pointRecord.intensity = static_cast<unsigned short>(1.0);
		pointRecord.mask = 0;
		pointRecord.mask = 0;
		pointRecord.classification = (*points)[i].classification;
		pointRecord.scanAngle = 0;
		pointRecord.userData = 0;
		pointRecord.pointSourceID = 0;
		pointRecord.GPS = 0.0;
		pointRecord.red = 0xFFFF;
		pointRecord.green = 0xFFFF;
		pointRecord.blue = 0xFFFF;
		lasFile.write((char *)&pointRecord,sizeof(PointDataRecord));
	}

	lasFile.close();
	std::cout <<"las file is ok." << std::endl;
	return true;
}

bool CLasOperator::savePoint3dcAsLas(const char *file_name, std::vector<Point3DC> *point3dc){
	// Open a las file
	std::fstream lasFile;
	lasFile.open(file_name,std::ios::out|std::ios::binary);
	if(lasFile.fail())
		return false;

	// Create PulicHeaderBlock
	PublicHeaderBlock publicHeader;
	sprintf(publicHeader.fileSign,"LASF");
	publicHeader.fileSourceID = 0;
	publicHeader.reserved = 0;
	publicHeader.GUID1 = 0;
	publicHeader.GUID2 = 0;
	publicHeader.GUID3 = 0;
	memset(publicHeader.GUID4,0,sizeof(publicHeader.GUID4));

	publicHeader.versionMajor = 1;
	publicHeader.versionMinor = 2;
	sprintf(publicHeader.systemID,"ddd");
	sprintf(publicHeader.GenSoft,"Allen Software");
	publicHeader.creationDay = 1;
	publicHeader.creationYear = 2013;
	publicHeader.headerSize = sizeof(PublicHeaderBlock);
	publicHeader.offsetToData = sizeof(PublicHeaderBlock);
	publicHeader.varRecordNum = 0;
	publicHeader.dataFormat = 3;
	publicHeader.pointRecordLen = sizeof(PointDataRecord);
	publicHeader.pointRecordNum = point3dc->size();
	memset(publicHeader.returnPointNum,0,sizeof(publicHeader.returnPointNum));

	publicHeader.maxX = -DBL_MAX;
	publicHeader.minX = DBL_MAX;
	publicHeader.maxY = -DBL_MAX;
	publicHeader.minY = DBL_MAX;
	publicHeader.maxZ = -DBL_MAX;
	publicHeader.minZ = DBL_MAX;
	for(size_t i=0;i<point3dc->size();++i){
		if((*point3dc)[i].x>publicHeader.maxX)
			publicHeader.maxX = (*point3dc)[i].x;
		if((*point3dc)[i].x<publicHeader.minX)
			publicHeader.minX = (*point3dc)[i].x;
		if((*point3dc)[i].y>publicHeader.maxY)
			publicHeader.maxY = (*point3dc)[i].y;
		if((*point3dc)[i].y<publicHeader.minY)
			publicHeader.minY = (*point3dc)[i].y;
		if((*point3dc)[i].z>publicHeader.maxZ)
			publicHeader.maxZ = (*point3dc)[i].z;
		if((*point3dc)[i].z<publicHeader.minZ)
			publicHeader.minZ = (*point3dc)[i].z;
	}

	double scaleX = publicHeader.maxX-publicHeader.minX>DBL_EPSILON?publicHeader.maxX-publicHeader.minX:DBL_EPSILON;
	double scaleY = publicHeader.maxY-publicHeader.minY>DBL_EPSILON?publicHeader.maxY-publicHeader.minY:DBL_EPSILON;
	double scaleZ = publicHeader.maxZ-publicHeader.minZ>DBL_EPSILON?publicHeader.maxZ-publicHeader.minZ:DBL_EPSILON;
	publicHeader.xScale = 1.0e-9 * scaleX;
	publicHeader.yScale = 1.0e-9 * scaleY;
	publicHeader.zScale = 1.0e-9 * scaleZ;
	publicHeader.xOffset = publicHeader.minX;
	publicHeader.yOffset = publicHeader.minY;
	publicHeader.zOffset = publicHeader.minZ;

	// Write pulbic header block
	lasFile.write((char *)(&publicHeader),sizeof(PublicHeaderBlock));

	// Export data points
	for(size_t i=0;i<point3dc->size();++i){
		PointDataRecord pointRecord;
		pointRecord.x = static_cast<long>(((*point3dc)[i].x-publicHeader.xOffset)/publicHeader.xScale);
		pointRecord.y = static_cast<long>(((*point3dc)[i].y-publicHeader.yOffset)/publicHeader.yScale);
		pointRecord.z = static_cast<long>(((*point3dc)[i].z-publicHeader.zOffset)/publicHeader.zScale);
		//pointRecord.intensity = static_cast<unsigned short>((*points)[i].intensity);
		pointRecord.intensity = static_cast<unsigned short>(1.0);
		pointRecord.mask = 0;
		pointRecord.mask = 0;
		//pointRecord.classification = 
		pointRecord.scanAngle = 0;
		pointRecord.userData = 0;
		pointRecord.pointSourceID = 0;
		pointRecord.GPS = 0.0;
		pointRecord.red = (*point3dc)[i].red;
		pointRecord.green = (*point3dc)[i].green;
		pointRecord.blue = (*point3dc)[i].blue;
		lasFile.write((char *)&pointRecord,sizeof(PointDataRecord));
	}

	lasFile.close();
	std::cout <<"las file is ok." << std::endl;
	return true;
}

std::string CLasOperator::getFileName(){
	int pos = 0;
	while(filename[pos+1] != '.')++pos;
	string name = filename.substr(0,pos+1);
	return name;
}

unsigned long CLasOperator::getRecordNum(){
	return pHeader.pointRecordNum;
}

void CLasOperator::replacePointData(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> tgtPatch, OriginXy const oriXy, std::vector<int> &const pointIdx){
	vector<int> countCloud;//每个点云的大小计数
	for(size_t i = 0; i < oriXy.xBlock*oriXy.yBlock; ++i)
		countCloud.push_back(0);
	//更改配准后发生变化的值
	//最值
	pHeader.maxX = -DBL_MAX;
	pHeader.minX = DBL_MAX;
	pHeader.maxY = -DBL_MAX;
	pHeader.minY = DBL_MAX;
	pHeader.maxZ = -DBL_MAX;
	pHeader.minZ = DBL_MAX;
	for(size_t i = 0; i < pHeader.pointRecordNum; ++i){
		int idx = pointIdx[i];//idx:这个点分在了第几片patch里
		//这里的float转化成double 小数位增加了一些
// 		if(tgtPatch[idx]->size() == 0){//在配准那一步，当有一方为空时，没有执行配准，所以没有给tgtPatch[idx]赋值（这个问题还没解决）
// 			continue;
// 		}
// 		//因为toPatchs()函数里面减去了最值，这里加上
		double x = (double)(tgtPatch[idx]->points[countCloud[idx]].x) + oriXy.minX;//countCloud[idx]:第idx片点云访问到了第countCloud[idx]个点
		double y = (double)(tgtPatch[idx]->points[countCloud[idx]].y) + oriXy.minY;
		double z = (double)(tgtPatch[idx]->points[countCloud[idx]++].z) + oriXy.minZ;
		//point3d
		point3d[i].x = x;
		point3d[i].y = y;
		point3d[i].z = z;
		if(pHeader.maxX < x)pHeader.maxX = x;
		if(pHeader.minX > x)pHeader.minX = x;
		if(pHeader.maxY < x)pHeader.maxY = y;
		if(pHeader.minY > x)pHeader.minY = y;
		if(pHeader.maxZ < x)pHeader.maxZ = z;
		if(pHeader.minZ > x)pHeader.minZ = z;
	}
}

/*================================
HPDpointclouddataread函数简介
功能：某个点邻域信息的可视化（彩色化）
输入形参1：const std::string name为点云文件名
输入形参4：选择是las文件还是pcd文件，其中las文件为1选项（默认），其他为pcb
输入形参2：一个智能指针用于存储pointXYZ类型的数据(float)
输入形参3：一个容器存储point3d类型数据(double)
输出：满状态的double型和float型数据用于记录点云每个点的笛卡尔系坐标位置
=================================*/
void HPDpointclouddataread(const std::string name,pcl::PointCloud<pcl::PointXYZ>::Ptr & las_cloud,std::vector<Point3D> & point3d,int type){
	if (type==1)//读取点云las文件
   {
	CLasOperator lasptr;
	//调用las读取函数
	if(!lasptr.readLasFile(name.c_str())){
		std::cout<<"Read file failed!"<<std::endl;
	exit(0);
	}
	std::cout<<"Read file succeed!"<<std::endl;
	//获取点云中的点数量
	int num = lasptr.pointNum();
	point3d = lasptr.getPointData();
	//++++++++++++++++++++++++++转为pcd++++++++++++++++++++++++++
// 创建PCL类点云和PCD格式
las_cloud->width=num;
las_cloud->height=1;
las_cloud->is_dense=false;
las_cloud->points.resize(las_cloud->width*las_cloud->height);
//将las数据复制给pcl类
for(size_t i=0;i<num;++i)
{
las_cloud->points[i].x=float(point3d[i].x);
las_cloud->points[i].y=float(point3d[i].y);
las_cloud->points[i].z=float(point3d[i].z);
}
	}
	else//读PCD文件
	{
		//*打开PCD文件	
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(name,*las_cloud)==-1){
PCL_ERROR("Couldn't read file test_pcd.pcd\n");
exit(0);
}
//计算时只要point3d，而调用pcl需要pointxyz，转为point3d因为pointXYZ精度不够。需要double型的
point3d.resize(las_cloud->points.size());
//将las数据复制给point3d
for(size_t i=0;i!=point3d.size();++i)
{
point3d[i].x=double(las_cloud->points[i].x);
point3d[i].y=double(las_cloud->points[i].y);
point3d[i].z=double(las_cloud->points[i].z);}//end for
   }//end small if
	std::cout<<"finish reading."<<std::endl;
}

/*======================================
Tranlasorpcdfile函数简介
将大点云切块
Point3D格式转换成pcl::PointCloud<pcl::PointXYZ>::Ptr
========================================*/
void toPatchs(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & p_cloud,  CLasOperator & lo, const OriginXy oriXy, std::vector<int> &pointIdx){//函数里面取一个切割原点(左上角)
 
	int nBlock =  oriXy.xBlock * oriXy.yBlock;//总分块数

	for(size_t i = 0; i < nBlock; ++i){
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		p_cloud.push_back(cloud);
	}
	vector<int> countCloud;//每个点云的大小计数
	for(size_t i = 0; i < nBlock; ++i)
		countCloud.push_back(0);

	std::vector<Point3D> points = lo.getPointData();
	//必须先申请内存大小(就是先填头信息的宽度：点的个数)，再存入点云
	for(size_t i = 0; i < points.size(); ++i){
		//int idx = int((point3d[i].x - cloudHeader.minX)/10 + 1) + int((point3d[i].y - cloudHeader.minY)/10) * xc - 1;//判断存储在第几个点云里
		//这样的式子太长就分开写，以免发生隐式类型转换而没有发现
		double x = points[i].x;
		double y = points[i].y;
		int xx = (x - oriXy.x)/oriXy.step + 1;
		if(x - oriXy.x < oriXy.step)xx = 1;//算在第一块，包括负数
		if(x - oriXy.x >= oriXy.step*oriXy.xBlock)xx = oriXy.xBlock;//算在最后一块,包括后面超过最后一块宽度的点
		int yy = (y - oriXy.y)/oriXy.step + 1;
		if(y - oriXy.y < oriXy.step)yy = 1;//算在第一块，包括负数
		if(y - oriXy.y >= oriXy.step*oriXy.yBlock)yy = oriXy.yBlock;
		int idx = xx + (yy - 1) * oriXy.xBlock - 1;//算出存储在第几块点云里
		countCloud[idx] += 1;//第idx片点云的点云个数加1
		pointIdx.push_back(idx);
	}
	for(size_t i = 0; i < nBlock; ++i){//每片点云的头信息
		//p_cloud[i]->width = countCloud[i];
		p_cloud[i]->width = countCloud[i];
		countCloud[i] = 0;//赋值为0 ，以便下面使用
		p_cloud[i]->height = 1;
		p_cloud[i]->is_dense = false;
		p_cloud[i]->points.resize(p_cloud[i]->width * p_cloud[i]->height);
	}
	for(size_t i = 0; i < points.size(); ++i){//遍历所有的点,并且分块
		//int idx = int((point3d[i].x - cloudHeader.minX)/10 + 1) + int((point3d[i].y - cloudHeader.minY)/10) * xc - 1;//存储在第几个点云里
		double x = points[i].x;
		double y = points[i].y;
		int xx = (x - oriXy.x)/oriXy.step + 1;
		if(x - oriXy.x < oriXy.step)xx = 1;//算在第一块，包括负数
		if(x - oriXy.x >= oriXy.step*oriXy.xBlock)xx = oriXy.xBlock;//算在最后一块,包括后面超过最后一块宽度的点
		int yy = (y - oriXy.y)/oriXy.step + 1;
		if(y - oriXy.y < oriXy.step)yy = 1;//算在第一块，包括负数
		if(y - oriXy.y >= oriXy.step*oriXy.yBlock)yy = oriXy.yBlock;
		int idx = xx + (yy - 1) * oriXy.xBlock - 1;//算出存储在第几块点云里

		//这里的points里面的x,y,z是double型数据，但是p_cloud->points的x,y,z是float型数据,所以会丢失精度
		//这里是不是两片配准融合后的点云不能完全遮盖第一片点云的原因（第一片点云坐标没有变动，理应被融合后的点云完全遮盖）
		//在CloudCompare中观察到应该是这样
		p_cloud[idx]->points[countCloud[idx]].x = float(points[i].x - oriXy.minX);//这里的double到float精度丢失太严重，怎么办
		p_cloud[idx]->points[countCloud[idx]].y = float(points[i].y - oriXy.minY);
		p_cloud[idx]->points[countCloud[idx]].z = float(points[i].z - oriXy.minZ);
		countCloud[idx] += 1;//第idx片点云的点云个数加1
	}
	//存储分块后的点云到硬盘
	/*string filename = lo.getFileName();
	for(size_t i = 0; i < nBlock; ++i){
	stringstream ss;
	string s;
	ss <<  filename + '_';
	ss << i;
	ss >> s;
	string name = s + ".pcd";
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>(name,*p_cloud[i]);
	}*/
	
}
/*======================================
HPDpointcloudallread函数简介
输入：const std::string name 加后缀的文件名
      pcl::PointCloud<pcl::PointXYZI>::Ptr & las_cloud PCD格式的点云 pcl用
	  std::vector<Point3DI> & pointall Point3DI格式的点云 las用
	  int type 读取pcd还是las文件
输出：带强度信息的点云
=========================================*/
void HPDpointcloudallread(const std::string name,pcl::PointCloud<pcl::PointXYZI>::Ptr & las_cloud,std::vector<Point3DI> & pointall,int type){
	if (type==1)//读取点云las文件
   {
	CLasOperator lasptr;
	//调用las读取函数
	if(!lasptr.readLasFile(name.c_str())){
		std::cout<<"Read file failed!"<<std::endl;
	exit(0);
	}
	std::cout<<"Read file succeed!"<<std::endl;
	//清空
	pointall.clear();
	//获取点云中的点数量
	int num = lasptr.pointNum();
	//读取PointDataRecord格式下的intensity，注该格式下的坐标精度极差
	std::vector<PointDataRecord> pointrecords;
	pointrecords = lasptr.getPointRecords();
	//读取point3d下的三维坐标值
	std::vector<Point3D> point3d;
	point3d=lasptr.getPointData();
	//合二为一
	Point3DI onepoint3di={0.0,0.0,0.0,0,0};
	for(size_t i=0;i!=point3d.size();i++){
		onepoint3di.x=point3d[i].x;
		onepoint3di.y=point3d[i].y;
		onepoint3di.z=point3d[i].z;
		onepoint3di.intensity=pointrecords[i].intensity;
		pointall.push_back(onepoint3di);
	}
	//+++++++++++++转为pcd
    // 创建PCL类点云和PCD格式
    las_cloud->width=num;
    las_cloud->height=1;
    las_cloud->is_dense=false;
    las_cloud->points.resize(las_cloud->width*las_cloud->height);
    //将las数据复制给pcl类
    for(size_t i=0;i<num;++i){
    las_cloud->points[i].x=float(pointall [i].x);
    las_cloud->points[i].y=float(pointall [i].y);
    las_cloud->points[i].z=float(pointall [i].z);
    las_cloud->points[i].intensity=float(pointall[i].intensity);
	}//end for
	
	//*************************读PCD文件的模式**************************
	}else{
		//*打开PCD文件	
    if(pcl::io::loadPCDFile<pcl::PointXYZI>(name,*las_cloud)==-1){
       PCL_ERROR("Couldn't read file test_pcd.pcd\n");
       exit(0);
      }
//计算时只要pointall，而调用pcl需要pointxyzI，转为pointall因为pointXYZI精度不够。需要double型的
       pointall.resize(las_cloud->points.size());
//将las数据复制给pointall
       for(size_t i=0;i!=pointall.size();++i){
           pointall[i].x=double(las_cloud->points[i].x);
           pointall[i].y=double(las_cloud->points[i].y);
           pointall[i].z=double(las_cloud->points[i].z);
           pointall[i].intensity=unsigned short(las_cloud->points[i].intensity);}//end for
       }//end small if
	std::cout<<"finish reading."<<std::endl;
}