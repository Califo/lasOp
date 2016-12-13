
#include "LasOperator.h"

#include <string> 
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/cloud_viewer.h>
// using pcl::visualization::PointCloudColorHandlerGenericField;
// using pcl::visualization::PointCloudColorHandlerCustom;
//简单类型定义
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
//这是一个辅助教程，因此我们可以负担全局变量
	//创建可视化工具
// pcl::visualization::PCLVisualizer *p;
	//定义左右视点
//int vp_1, vp_2;
//处理点云的方便的结构定义
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;
  PCD() : cloud (new PointCloud) {};
};
struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};
//以< x, y, z, curvature >形式定义一个新的点
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    //定义尺寸值
    nr_dimensions_ = 4;
  }
  //覆盖copyToFloatArray方法来定义我们的特征矢量
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};
////////////////////////////////////////////////////////////////////////////////
/** 在可视化窗口的第一视点显示源点云和目标点云
*
 */
// void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source){
// 	p->removePointCloud("vp1_target");
// 	p->removePointCloud("vp1_source");
// 	PointCloudColorHandlerCustom<PointT> tgt_h(cloud_target, 0, 255, 0);
// 	PointCloudColorHandlerCustom<PointT> src_h(cloud_source, 255, 0, 0);
// 	p->addPointCloud(cloud_target, tgt_h, "vp1_target", vp_1);
// 	p->addPointCloud(cloud_source, src_h, "vp1_source", vp_1);
// 	p->spin();//Calls the interactor and runs an internal loop
// }
////////////////////////////////////////////////////////////////////////////////
/**在可视化窗口的第二视点显示源点云和目标点云
 *
 */
// void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source){
// 	p->removePointCloud("source");
// 	p->removePointCloud("target");
// 	PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler(cloud_target, "curvature");
// 	if (!tgt_color_handler.isCapable ())
// 		PCL_WARN ("Cannot create curvature color handler!");
// 	PointCloudColorHandlerGenericField<PointNormalT> src_color_handler(cloud_source, "curvature");
// 	if (!src_color_handler.isCapable ())
// 		PCL_WARN ("Cannot create curvature color handler!");
// 	p->addPointCloud(cloud_target, tgt_color_handler, "target", vp_2);
// 	p->addPointCloud(cloud_source, src_color_handler, "source", vp_2);
// 	p->spinOnce();
// }
////////////////////////////////////////////////////////////////////////////////
/**加载一组我们想要匹配在一起的PCD文件
  * 参数argc是参数的数量 (pass from main ())
  *参数 argv 实际的命令行参数 (pass from main ())
  *参数models点云数据集的合成矢量
  */
void loadData(int argc,char **argv,std::vector<PCD, Eigen::aligned_allocator<PCD>> &data){
	std::string ext(".pcd");
	for(int i = 1; i<argc; ++i){//为什么从1而不是从0开始：argv[0]:.exe argv[1]:a.pcd argv[2]:b.pcd ...
		std::string fname = std::string(argv[i]);
		if(fname.size()<=ext.size())//若文件名为空(只有扩展名)，则跳过
			continue;
		std::transform(fname.begin(),fname.end(),fname.begin(),(int(*)(int))tolower);//转换为小写
		if(fname.compare(fname.size()-ext.size(), ext.size(), ext) == 0){//比较扩展名是否与ext相同
		//str1的子串（从索引3开始，包含4个字符）与str2进行比较  //if(str1.compare(3,4,str2)==0)
			PCD d;
			d.f_name = argv[i];
			pcl::io::loadPCDFile(d.f_name,*d.cloud);//保存到d中
			//移除NAN点
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*d.cloud,*d.cloud,indices);
			data.push_back(d);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////// 
////////////////////////////////////////////////////////////////////////////////

std::vector<Eigen::Matrix4f> vec_mattrix;//用于存储每对点云第一次配准的转换矩阵

bool pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false){
	PointCloud::Ptr src(new PointCloud);
	PointCloud::Ptr tgt(new PointCloud);
	//是否需要下采样（以防大数据集）
	pcl::VoxelGrid<PointT> grid;
	if(cloud_tgt->width < 100000){//小于这个数就不下采样了
		downsample = false;
	}
	if(downsample){
		std::cout<< "downsample" << std::endl;
		grid.setLeafSize(0.05, 0.05, 0.05);//单位：m
//		grid.setLeafSize(0.10, 0.10, 0.10);
		grid.setInputCloud(cloud_src);
		grid.filter(*src);
		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
	}
	else{
		src = cloud_src;
		tgt = cloud_tgt;
	}
	//计算曲面法线和曲率
	PointCloudWithNormals::Ptr point_with_normals_src(new PointCloudWithNormals);
	PointCloudWithNormals::Ptr point_with_normals_tgt(new PointCloudWithNormals);
	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);
	norm_est.setInputCloud(src);//....
	norm_est.compute(*point_with_normals_src);
	pcl::copyPointCloud(*src, *point_with_normals_src);
	norm_est.setInputCloud(tgt);//....
	norm_est.compute(*point_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *point_with_normals_tgt);
	//
	//自定义点的表示？？？
	MyPointRepresentation point_representation;
	float alpha[4] = {1.0, 1.0, 1.0, 1.0};
	point_representation.setRescaleValues(alpha);
	//
	//准备就绪，创建ICP对象，进行配准
	pcl::IterativeClosestPointNonLinear<PointNormalT,PointNormalT> reg;
	reg.setTransformationEpsilon(1e-7);//收敛判断条件,越小精度越高
	reg.setMaxCorrespondenceDistance(0.05);//将两个对应关系(src-tgt)之间的最大距离设置为0.1米，大于此值得点对不予考虑（注意：根据数据集的实际来调整）
	reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));//设置点表示
	reg.setInputCloud(point_with_normals_src);
	reg.setInputTarget(point_with_normals_tgt);
	//
	Eigen::Matrix4f prev, targetToSource, Ti = Eigen::Matrix4f::Identity();
	PointCloudWithNormals::Ptr reg_result = point_with_normals_src;
	reg.setMaximumIterations(5);//限制内部优化运行最大迭代次数为2，即每内部迭代2次就认为收敛，停止内部迭代
	//手动迭代（）次
	for(int i = 0; i < 3; ++i){
		PCL_INFO("Iteration Nr. %d\n", i);
		point_with_normals_src = reg_result;//为了可视化保存点云
		reg.setInputCloud(point_with_normals_src);
//		std::cout << "align begin" << std::endl;//
		reg.align(*reg_result);
//		std::cout << "align over" << std::endl;
		if(reg.hasConverged() == 0){//配准失败返回false（失败的原因？？？）
			return false;
		}
		//hasConverged()值为1和0分别表示配准成功和失败；getFitnessScore()表示源点云与目标点云距离平方和（只考虑小于某阈值的点对）
		std::cout << "has converged:" << reg.hasConverged() << std::endl << "score: " <<reg.getFitnessScore() << std::endl;
		Ti = reg.getFinalTransformation()*Ti;//累积转换
		if(fabs((reg.getLastIncrementalTransformation()-prev).sum()) < reg.getTransformationEpsilon())//如果这次转换与之前转化之差小于阈值，则减小最大对应距离
			reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);
		prev = reg.getLastIncrementalTransformation();
		//showCloudsRight(point_with_normals_tgt, point_with_normals_src);//可视化
	}
	targetToSource = Ti.inverse();//Ti为源点云到目标点云的变换，求逆得到目标点云到源点云的变换
	pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);//一个仿射变换，参数(cloud_in, cloud_out, transform)
	//p->removePointCloud("source");
	//p->removePointCloud("target");
	//PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
	//PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
	//p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
	//p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);
	//PCL_INFO ("Press q to continue the registration.\n");
	//p->spin ();
	//p->removePointCloud ("source"); 
	//p->removePointCloud ("target");
	//添加源点云到转换目标
	//*output += *cloud_src;//用于返回
	final_transform = targetToSource;//用于返回
	return true;
}

void loadLasData(int argc,char **argv,std::vector<CLasOperator> &vec_lo){
	std::string ext(".las");
	for(size_t i = 1; i<argc; ++i){
		std::string	fname = std::string(argv[i]);
		if(fname.size() <= ext.size())continue;
		std::transform(fname.begin(), fname.end(), fname.begin(), ::tolower);
		if(fname.compare(fname.size() - ext.size(), ext.size(), ext) == 0){
			vec_lo[i-1].readLasFile(argv[i]);
		}
	}
}

void getPatchInfo(PublicHeaderBlock &publicHeader, OriginXy &oriXy, int step){
	//由Point3D的坐标转化成PointDataRecord的坐标
// 	double minX = static_cast<long>((publicHeader.minX - publicHeader.xOffset)/publicHeader.xScale);
// 	double maxX = static_cast<long>((publicHeader.maxX - publicHeader.xOffset)/publicHeader.xScale);
// 	double minY = static_cast<long>((publicHeader.minY - publicHeader.yOffset)/publicHeader.yScale);
// 	double maxY = static_cast<long>((publicHeader.maxY - publicHeader.yOffset)/publicHeader.yScale);

	//用Point3D的坐标系(绝对坐标系)
	double minX = publicHeader.minX;
	double maxX = publicHeader.maxX;
	double minY = publicHeader.minY;
	double maxY = publicHeader.maxY;

	//步长
	oriXy.step = step;
	//分割原点
	oriXy.x = (int)minX/10*10;
	if(minX < 0)oriXy.x -= 10;//负数向下取整绝对值变大
	oriXy.y = (int)minY/10*10;
	if(minY < 0)oriXy.y -= 10;
	//xy方向上各有几块
	oriXy.xBlock = (maxX/10*10 - oriXy.x)/oriXy.step + 1;
	oriXy.yBlock = (maxY/10*10 - oriXy.y)/oriXy.step + 1;

	oriXy.minX = publicHeader.minX;
	oriXy.minY = publicHeader.minY;
	oriXy.minZ = publicHeader.minZ;
}

//是否矩阵顶点
bool isVertex(OriginXy ori, int x, int y){
	if((x==0 && y==0) || (x==0 && y==ori.xBlock-1) || (x==ori.yBlock-1 && y==ori.xBlock-1) || (x==ori.yBlock-1 && y==0))
		return true;
	return false;
}
//得到空间上最相近的有效配准矩阵的索引
int getClosestTransformIdx(OriginXy ori, int n, std::vector<bool> vec_isAlign){
	int y = n % ori.xBlock;
	int x = (n - y)/ori.yBlock;
	int isVer = 0;//访问到四个顶点的个数
	if(isVertex(ori,x,y))//先判断出发点是不是顶点
		++isVer;
	bool get = false;//找到转换矩阵
	int base = 0;
	while(isVer < 4){
		for(size_t i = 1; i <= base+1; ++i){
			++y;
			if(x > -1 && x < ori.yBlock && y > -1 && y <ori.xBlock){//在有效访问区间内
				if(vec_isAlign[x*ori.xBlock + y] == true){//找到
					return x * ori.xBlock + y;
				}else if(isVertex(ori,x,y)){//访问到矩阵顶点
					++isVer;
				}
			}
		}
		for(size_t i = 1; i <= base+1; ++i){
			++x;
			if(x > -1 && x < ori.yBlock && y > -1 && y <ori.xBlock){//在有效访问区间内
				if(vec_isAlign[x*ori.xBlock + y] == true){//找到
					return x * ori.xBlock + y;
				}else if(isVertex(ori,x,y)){//访问到矩阵顶点
					++isVer;
				}
			}
		}
		for(size_t i = 1; i <= base+2; ++i){
			--y;
			if(x > -1 && x < ori.yBlock && y > -1 && y <ori.xBlock){//在有效访问区间内
				if(vec_isAlign[x*ori.xBlock + y] == true){//找到
					return x * ori.xBlock + y;
				}else if(isVertex(ori,x,y)){//访问到矩阵顶点
					++isVer;
				}
			}
		}
		for(size_t i = 1; i <= base+2; ++i){
			--x;
			if(x > -1 && x < ori.yBlock && y > -1 && y <ori.xBlock){//在有效访问区间内
				if(vec_isAlign[x*ori.xBlock + y] == true){//找到
					return x * ori.xBlock + y;
				}else if(isVertex(ori,x,y)){//访问到矩阵顶点
					++isVer;
				}
			}
		}
		base += 2;
	}
	return n;//没有找到则返回本身的索引
}

std::vector<Eigen::Matrix4f> vec_transform;//存储有效的变换矩阵

void patchToPairAlign(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &p1_cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &p2_cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &tgtPatch, OriginXy oriXy){
	std::vector<bool> vec_isAlign(oriXy.xBlock*oriXy.yBlock, false);
	std::vector<bool> vec_isNull(oriXy.xBlock*oriXy.yBlock, false);
	Eigen::Matrix4f pairTransform = Eigen::Matrix4f::Identity();//变换矩阵
	//先用单位矩阵填充变换矩阵，
	for(size_t i = 0; i < oriXy.xBlock*oriXy.yBlock; ++i)
		vec_transform.push_back(pairTransform);
	for(size_t i = 0; i < oriXy.xBlock*oriXy.yBlock; ++i){
		std::cout << "第" << i+1 << "/" << oriXy.xBlock * oriXy.yBlock << "对开始" << std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTmp(new pcl::PointCloud<pcl::PointXYZ>);
		tgtPatch.push_back(cloudTmp);
		//showCloudsLeft(p1_cloud[i],p2_cloud[i]);
		if(p2_cloud[i]->width == 0){//目标点云为空，则不需要处理
			vec_isAlign[i] = false;
			vec_isNull[i] = true;
			std::cout << "第" << i+1 << "/" << oriXy.xBlock * oriXy.yBlock << "对不需要配准" << std::endl << std::endl;
		}else if(p1_cloud[i]->width < 10 || p2_cloud[i]->width < 10){//点太少时（遇到过src里面只有一个点），align函数报错，但是至少需要多少个点才行，暂时定10个以上。(kdtree_flann.hpp line136 见有道云笔记)
			std::cout << "第" << i+1 << "/" << oriXy.xBlock * oriXy.yBlock << "对配准失败" << std::endl << std::endl;
		}else{
			bool isAlign = pairAlign(p1_cloud[i], p2_cloud[i], tgtPatch[i], pairTransform, true);//根据需要可以进行下采样
			if(isAlign){
				vec_transform[i] = pairTransform;
				vec_isAlign[i] = true;
				std::cout << "第" << i+1 << "/" << oriXy.xBlock * oriXy.yBlock << "对配准完成" << std::endl << std::endl;
			}else{
				std::cout << "第" << i+1 << "/" << oriXy.xBlock * oriXy.yBlock << "对配准失败" << std::endl << std::endl;
			}
		}
	}
	//如果有剩下没有完成配准的，则利用在空间上最相近的点云对的有效转换矩阵来配准
	std::cout << "达不到配准条件的点云对，借鉴附近的转换矩阵配准：" << std::endl;
	bool hasLeft = false;//是否有剩下需要配准的
	for(size_t i = 0; i < vec_isAlign.size(); ++i){
		if(vec_isAlign[i] == false && vec_isNull[i] == false){//没有配准且不为空的点云
			hasLeft = true;
			pairTransform = vec_transform[getClosestTransformIdx(oriXy, i, vec_isAlign)];
			pcl::transformPointCloud(*p2_cloud[i], *tgtPatch[i], pairTransform);
			std::cout << "第" << i+1 << "/" << oriXy.xBlock * oriXy.yBlock << "对配准完成" << std::endl << std::endl;
		}
	}
	if(!hasLeft)
		std::cout << "无" << std::endl;
}


//除了合并之后必须改变的信息（例如：大小，最值等）外，其它信息依据第一个扫描头的点云填写
bool saveDoubleLasFile(const char *file_name, CLasOperator &lo1, CLasOperator &lo2){
	// Open a las file
	std::fstream lasFile;
	lasFile.open(file_name,std::ios::out|std::ios::binary);
	if(lasFile.fail())
		return false;

	PublicHeaderBlock header1 = lo1.getPublicHeader();
	PublicHeaderBlock header2 = lo2.getPublicHeader();
	//
	// 
	// 
	//按照Point3D的坐标系
	//不清楚为什么las文件里面按照PointDataRecord的坐标系存储的坐标，但是最值却用的Point3D的坐标系：
	// 头文件里面存的是绝对坐标系（points）的信息，用于将相对坐标（pointRecord）还原，处理信息就用的绝对坐标系。
	// 头文件里面存的是绝对坐标系（points）的信息，用于将相对坐标（pointRecord）还原，处理信息就用的绝对坐标系。
	// 所以不能用原来信息头里面的有些信息不能用（例如：最值）
	// 
	// 
	//第一步
	// Create PulicHeaderBlock
	PublicHeaderBlock publicHeader;
	//sprintf(publicHeader.fileSign,header1.fileSign);
	memcpy(publicHeader.fileSign, header1.fileSign, sizeof(publicHeader.fileSign));
	publicHeader.fileSourceID = header1.fileSourceID;
	publicHeader.reserved = header1.reserved;
	publicHeader.GUID1 = header1.GUID1;
	publicHeader.GUID2 = header1.GUID2;
	publicHeader.GUID3 = header1.GUID3;
	//sprintf((char *)publicHeader.GUID4, (const char*)header1.GUID4);//转换了类型--------影响不大吧？--------------------------
	memcpy(publicHeader.GUID4, header1.GUID4, sizeof(publicHeader.GUID4));

	publicHeader.versionMajor = header1.versionMajor;
	publicHeader.versionMinor = header1.versionMinor;
	//sprintf(publicHeader.systemID,header1.systemID);
	memcpy(publicHeader.systemID, header1.systemID, sizeof(publicHeader.systemID));
	//sprintf(publicHeader.GenSoft,header1.GenSoft);
	memcpy(publicHeader.GenSoft, header1.GenSoft, sizeof(publicHeader.GenSoft));
	publicHeader.creationDay = header1.creationDay;
	publicHeader.creationYear = header1.creationYear;

	publicHeader.headerSize = sizeof(PublicHeaderBlock);
	publicHeader.offsetToData = sizeof(PublicHeaderBlock);
	publicHeader.varRecordNum = header1.varRecordNum + header2.varRecordNum;//用到了header1,header2-------------------
	publicHeader.dataFormat = header1.dataFormat;
	publicHeader.pointRecordLen = sizeof(PointDataRecord);//-------这一项每个las文件应该是一致的吧？--------------
	publicHeader.pointRecordNum = header1.pointRecordNum + header2.pointRecordNum;//用到了header1,header2-------------------
	memcpy(publicHeader.returnPointNum, header1.returnPointNum, sizeof(publicHeader.returnPointNum));

	//这里的最值是point3d坐标系（绝对坐标）的，lo2中的最值在replacePointData()函数中已经更新过，所以这里不用再遍历lo2中point3d确定最值了。而lo1中最值是没有变的。
	publicHeader.maxX = header1.maxX > header2.maxX ? header1.maxX : header2.maxX;
	publicHeader.minX = header1.minX < header2.minX ? header1.minX : header2.minX;
	publicHeader.maxY = header1.maxY > header2.maxY ? header1.maxY : header2.maxY;
	publicHeader.minY = header1.minY < header2.minY ? header1.minY : header2.minY;
	publicHeader.maxZ = header1.maxZ > header2.maxZ ? header1.maxZ : header2.maxZ;
	publicHeader.minZ = header1.minZ < header2.minZ ? header1.minZ : header2.minZ;

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

	//第二步
	//   lo1
	std::vector<Point3D> pointData1 =  lo1.getPointData();
	std::vector<PointDataRecord> pointRecord1 =  lo1.getPointRecords();
	for(size_t i = 0; i < pointData1.size(); ++i){
		PointDataRecord pointRecord;
		pointRecord.x = static_cast<long>((pointData1[i].x-publicHeader.xOffset)/publicHeader.xScale);
		pointRecord.y = static_cast<long>((pointData1[i].y-publicHeader.yOffset)/publicHeader.yScale);
		pointRecord.z = static_cast<long>((pointData1[i].z-publicHeader.zOffset)/publicHeader.zScale);

		pointRecord.intensity = pointRecord1[i].intensity;
		pointRecord.mask = pointRecord1[i].mask;

		pointRecord.classification = pointRecord1[i].classification;
		pointRecord.scanAngle = pointRecord1[i].scanAngle;
		pointRecord.userData = pointRecord1[i].userData;
		pointRecord.pointSourceID = pointRecord1[i].pointSourceID;
		pointRecord.GPS = pointRecord1[i].GPS;
		pointRecord.red = pointRecord1[i].red;
		pointRecord.green = pointRecord1[i].green;
		pointRecord.blue = pointRecord1[i].blue;
		lasFile.write((char *)&pointRecord,sizeof(PointDataRecord));
		if(i % 10000000 == 0){
			std::cout << "lo1中已写入" << i << std::endl;
		}
	}
	std::cout << std::endl << std::endl;
	// lo2 Point3D转化成PointRecord坐标系
	std::vector<Point3D> pointData2 =  lo2.getPointData();
	std::vector<PointDataRecord> pointRecord2 =  lo2.getPointRecords();
	for(size_t i = 0; i < pointData2.size(); ++i){
		PointDataRecord pointRecord;
		pointRecord.x = static_cast<long>((pointData2[i].x-publicHeader.xOffset)/publicHeader.xScale);
		pointRecord.y = static_cast<long>((pointData2[i].y-publicHeader.yOffset)/publicHeader.yScale);
		pointRecord.z = static_cast<long>((pointData2[i].z-publicHeader.zOffset)/publicHeader.zScale);

		pointRecord.intensity = pointRecord2[i].intensity;
		pointRecord.mask = pointRecord2[i].mask;

		pointRecord.classification = pointRecord2[i].classification;
		pointRecord.scanAngle = pointRecord2[i].scanAngle;
		pointRecord.userData = pointRecord2[i].userData;
		pointRecord.pointSourceID = pointRecord2[i].pointSourceID;
		pointRecord.GPS = pointRecord2[i].GPS;
		pointRecord.red = pointRecord2[i].red;
		pointRecord.green = pointRecord2[i].green;
		pointRecord.blue = pointRecord2[i].blue;
		lasFile.write((char *)&pointRecord,sizeof(PointDataRecord));
		if(i % 10000000 == 0){
			std::cout << "lo2中已写入" << i << std::endl;
		}
	}

	lasFile.close();
	std::cout << "las file(two-in-one) is OK" << std::endl;
	return true;
}


void saveSomePatchsAsLas(const char *file_name, CLasOperator &lo_somePatchs, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &tgtPatch, std::vector<int> const vec_idx, OriginXy const oriXy){
	std::vector<Point3DC> vec_point3dc;
	for(size_t n = 0; n < vec_idx.size(); ++n){
		//if(n > 4)break;//多于五个patch就不要了
		int r,g,b;
		switch (n)
		{
		case 0:
			r = 255;
			g = 0;
			b = 0;
			break;
		case 1:
			r = 0;
			g = 255;
			b = 0;
			break;
		case 2:
			r = 0;
			g = 0;
			b = 255;
			break;
		case 3:
			r = 255;
			g = 255;
			b = 0;
			break;
		case 4:
			r = 0;
			g = 255;
			b = 255;
			break;
		default:
			r = 255;
			g = 255;
			b = 255;
			break;
		}
		int idx = vec_idx[n];
		for(size_t i = 0; i < tgtPatch[idx]->size(); ++i){
			Point3DC point;
			point.x = (double)(tgtPatch[idx]->points[i].x) + oriXy.minX;
			point.y = (double)(tgtPatch[idx]->points[i].y) + oriXy.minY;
			point.z = (double)(tgtPatch[idx]->points[i].z) + oriXy.minZ;
			point.red = r;
			point.green = g;
			point.blue = b;
			vec_point3dc.push_back(point);
		}
	}
	std::cout << "开始存入patch" << std::endl;
	lo_somePatchs.savePoint3dcAsLas(file_name, &vec_point3dc);
	std::cout << "patch存入完成" << std::endl;
}

void rigidTransform(const char * file_name, const char * file_name_transform){
	CLasOperator lo;
	lo.readLasFile(file_name);
	std::vector<Point3DC> vec_point3dc = lo.getPoints();
	//没有加旋转
	//平移信息
	for(size_t i = 0; i < vec_point3dc.size(); ++i){
		vec_point3dc[i].x += 0.05;
		vec_point3dc[i].y += 0.03;
	}
	lo.savePoint3dcAsLas(file_name_transform, &vec_point3dc);
}

int main (int argc, char** argv)
{
	
//	rigidTransform("_scanner1-Cloud.las", "_scanner1-Cloud_rigidTransform.las");
//	char c5 = getchar();

	time_t start, stop;
	start = time(NULL);

	CLasOperator lo;
	std::vector<CLasOperator> vec_lo;
	vec_lo.push_back(lo);
	vec_lo.push_back(lo);

	//读入两片点云
	loadLasData(argc,argv,vec_lo);

	//切割信息
	OriginXy oriXy;
	int step = 50;//将分割步长设为整型方便，以免后面类型转换--------------------
	std::cout << "输入切块步长" << std::endl;
	std::cin >> step;
	std::cout << std::endl;
	getPatchInfo(vec_lo[0].getPublicHeader(), oriXy, step);
	
	//分块后的点云存储在p1_cloud,p2_cloud容器中
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> p1_cloud, p2_cloud;
	std::vector<int> pointIdx1;//存储每个点的去向（分到第几块点云）
	std::vector<int> pointIdx2;
	toPatchs(p1_cloud, vec_lo[0], oriXy, pointIdx1);	
	toPatchs(p2_cloud, vec_lo[1], oriXy, pointIdx2);

	//看看每片点云大小
	std::cout << std::endl;
	for(size_t i = 0; i < oriXy.xBlock*oriXy.yBlock; ++i){
		std::cout << i+1 << ":  " << p1_cloud[i]->width << "  " << p2_cloud[i]->width << std::endl;
	}

	//对应点云对配准
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> tgtPatch;//存储配准后的目标点云
	patchToPairAlign(p1_cloud, p2_cloud, tgtPatch, oriXy);
	std::cout << std::endl << std::endl << "配准完成" << std::endl;

	//查看变换矩阵
	std::vector<double> vec_dist;//存入平移变换距离
	for(size_t i = 0; i < vec_transform.size(); ++i){
		std::cout << i+1 << std::endl;
		std::cout << vec_transform[i] << std::endl;
		vec_dist.push_back(std::pow(vec_transform[i](0,3),2) + std::pow(vec_transform[i](1,3),2) + std::pow(vec_transform[i](2,3),2));
	}
	//按距离排序，按从大到小将原序号存入
	std::vector<int> vec_idx;
	for(size_t i = 0; i < vec_dist.size(); ++i){
		int maxIdx = i;
		for(size_t j = 0; j < vec_dist.size(); ++j){
			if(vec_dist[j] > vec_dist[maxIdx])
				maxIdx = j;
		}
		vec_dist[maxIdx] = -1;
		vec_idx.push_back(maxIdx);
	}
	//将变化程度较大的几块存成点云，程度由颜色区分
	CLasOperator lo_somePatchs;
	saveSomePatchsAsLas("E:\\PCL\\PCLdata\\somePatchs_xa.las", lo_somePatchs, tgtPatch, vec_idx, oriXy);

	///////////////////////

	//配准后lo2中的部分信息更新,要利用到索引pointIdx2 ！！！！！！！！！！！！！
	std::cout << "更新lo2中部分信息" << std::endl;
	vec_lo[1].replacePointData(tgtPatch, oriXy, pointIdx2);
	std::cout << "更新完成" << std::endl;

	tgtPatch.clear();//释放掉

	char c2 = getchar();
	std::cout << "跳过？" << std::endl;
	char c1 = getchar();

	//最后将第一片点云和配准后的第二片点云 一起写入磁盘（合并后部分信息要做更改）
	std::cout << "写入磁盘" << std::endl;
	saveDoubleLasFile("E:\\PCL\\PCLdata\\two_in_one_xa.las", vec_lo[0], vec_lo[1]);

	stop = time(NULL);
	std::cout << std::endl << "用时：  " << (stop - start) << std::endl;

	std::cout << "记录" << std::endl;
	char c3 = getchar();
	char c4 = getchar();
	return 0;
}

/* ]--- */
