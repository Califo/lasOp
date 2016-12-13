
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
//�����Ͷ���
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
//����һ�������̳̣�������ǿ��Ը���ȫ�ֱ���
	//�������ӻ�����
// pcl::visualization::PCLVisualizer *p;
	//���������ӵ�
//int vp_1, vp_2;
//������Ƶķ���Ľṹ����
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
//��< x, y, z, curvature >��ʽ����һ���µĵ�
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    //����ߴ�ֵ
    nr_dimensions_ = 4;
  }
  //����copyToFloatArray�������������ǵ�����ʸ��
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
/** �ڿ��ӻ����ڵĵ�һ�ӵ���ʾԴ���ƺ�Ŀ�����
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
/**�ڿ��ӻ����ڵĵڶ��ӵ���ʾԴ���ƺ�Ŀ�����
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
/**����һ��������Ҫƥ����һ���PCD�ļ�
  * ����argc�ǲ��������� (pass from main ())
  *���� argv ʵ�ʵ������в��� (pass from main ())
  *����models�������ݼ��ĺϳ�ʸ��
  */
void loadData(int argc,char **argv,std::vector<PCD, Eigen::aligned_allocator<PCD>> &data){
	std::string ext(".pcd");
	for(int i = 1; i<argc; ++i){//Ϊʲô��1�����Ǵ�0��ʼ��argv[0]:.exe argv[1]:a.pcd argv[2]:b.pcd ...
		std::string fname = std::string(argv[i]);
		if(fname.size()<=ext.size())//���ļ���Ϊ��(ֻ����չ��)��������
			continue;
		std::transform(fname.begin(),fname.end(),fname.begin(),(int(*)(int))tolower);//ת��ΪСд
		if(fname.compare(fname.size()-ext.size(), ext.size(), ext) == 0){//�Ƚ���չ���Ƿ���ext��ͬ
		//str1���Ӵ���������3��ʼ������4���ַ�����str2���бȽ�  //if(str1.compare(3,4,str2)==0)
			PCD d;
			d.f_name = argv[i];
			pcl::io::loadPCDFile(d.f_name,*d.cloud);//���浽d��
			//�Ƴ�NAN��
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*d.cloud,*d.cloud,indices);
			data.push_back(d);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////// 
////////////////////////////////////////////////////////////////////////////////

std::vector<Eigen::Matrix4f> vec_mattrix;//���ڴ洢ÿ�Ե��Ƶ�һ����׼��ת������

bool pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false){
	PointCloud::Ptr src(new PointCloud);
	PointCloud::Ptr tgt(new PointCloud);
	//�Ƿ���Ҫ�²������Է������ݼ���
	pcl::VoxelGrid<PointT> grid;
	if(cloud_tgt->width < 100000){//С��������Ͳ��²�����
		downsample = false;
	}
	if(downsample){
		std::cout<< "downsample" << std::endl;
		grid.setLeafSize(0.05, 0.05, 0.05);//��λ��m
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
	//�������淨�ߺ�����
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
	//�Զ����ı�ʾ������
	MyPointRepresentation point_representation;
	float alpha[4] = {1.0, 1.0, 1.0, 1.0};
	point_representation.setRescaleValues(alpha);
	//
	//׼������������ICP���󣬽�����׼
	pcl::IterativeClosestPointNonLinear<PointNormalT,PointNormalT> reg;
	reg.setTransformationEpsilon(1e-7);//�����ж�����,ԽС����Խ��
	reg.setMaxCorrespondenceDistance(0.05);//��������Ӧ��ϵ(src-tgt)֮�������������Ϊ0.1�ף����ڴ�ֵ�õ�Բ��迼�ǣ�ע�⣺�������ݼ���ʵ����������
	reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));//���õ��ʾ
	reg.setInputCloud(point_with_normals_src);
	reg.setInputTarget(point_with_normals_tgt);
	//
	Eigen::Matrix4f prev, targetToSource, Ti = Eigen::Matrix4f::Identity();
	PointCloudWithNormals::Ptr reg_result = point_with_normals_src;
	reg.setMaximumIterations(5);//�����ڲ��Ż���������������Ϊ2����ÿ�ڲ�����2�ξ���Ϊ������ֹͣ�ڲ�����
	//�ֶ�����������
	for(int i = 0; i < 3; ++i){
		PCL_INFO("Iteration Nr. %d\n", i);
		point_with_normals_src = reg_result;//Ϊ�˿��ӻ��������
		reg.setInputCloud(point_with_normals_src);
//		std::cout << "align begin" << std::endl;//
		reg.align(*reg_result);
//		std::cout << "align over" << std::endl;
		if(reg.hasConverged() == 0){//��׼ʧ�ܷ���false��ʧ�ܵ�ԭ�򣿣�����
			return false;
		}
		//hasConverged()ֵΪ1��0�ֱ��ʾ��׼�ɹ���ʧ�ܣ�getFitnessScore()��ʾԴ������Ŀ����ƾ���ƽ���ͣ�ֻ����С��ĳ��ֵ�ĵ�ԣ�
		std::cout << "has converged:" << reg.hasConverged() << std::endl << "score: " <<reg.getFitnessScore() << std::endl;
		Ti = reg.getFinalTransformation()*Ti;//�ۻ�ת��
		if(fabs((reg.getLastIncrementalTransformation()-prev).sum()) < reg.getTransformationEpsilon())//������ת����֮ǰת��֮��С����ֵ�����С����Ӧ����
			reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);
		prev = reg.getLastIncrementalTransformation();
		//showCloudsRight(point_with_normals_tgt, point_with_normals_src);//���ӻ�
	}
	targetToSource = Ti.inverse();//TiΪԴ���Ƶ�Ŀ����Ƶı任������õ�Ŀ����Ƶ�Դ���Ƶı任
	pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);//һ������任������(cloud_in, cloud_out, transform)
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
	//���Դ���Ƶ�ת��Ŀ��
	//*output += *cloud_src;//���ڷ���
	final_transform = targetToSource;//���ڷ���
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
	//��Point3D������ת����PointDataRecord������
// 	double minX = static_cast<long>((publicHeader.minX - publicHeader.xOffset)/publicHeader.xScale);
// 	double maxX = static_cast<long>((publicHeader.maxX - publicHeader.xOffset)/publicHeader.xScale);
// 	double minY = static_cast<long>((publicHeader.minY - publicHeader.yOffset)/publicHeader.yScale);
// 	double maxY = static_cast<long>((publicHeader.maxY - publicHeader.yOffset)/publicHeader.yScale);

	//��Point3D������ϵ(��������ϵ)
	double minX = publicHeader.minX;
	double maxX = publicHeader.maxX;
	double minY = publicHeader.minY;
	double maxY = publicHeader.maxY;

	//����
	oriXy.step = step;
	//�ָ�ԭ��
	oriXy.x = (int)minX/10*10;
	if(minX < 0)oriXy.x -= 10;//��������ȡ������ֵ���
	oriXy.y = (int)minY/10*10;
	if(minY < 0)oriXy.y -= 10;
	//xy�����ϸ��м���
	oriXy.xBlock = (maxX/10*10 - oriXy.x)/oriXy.step + 1;
	oriXy.yBlock = (maxY/10*10 - oriXy.y)/oriXy.step + 1;

	oriXy.minX = publicHeader.minX;
	oriXy.minY = publicHeader.minY;
	oriXy.minZ = publicHeader.minZ;
}

//�Ƿ���󶥵�
bool isVertex(OriginXy ori, int x, int y){
	if((x==0 && y==0) || (x==0 && y==ori.xBlock-1) || (x==ori.yBlock-1 && y==ori.xBlock-1) || (x==ori.yBlock-1 && y==0))
		return true;
	return false;
}
//�õ��ռ������������Ч��׼���������
int getClosestTransformIdx(OriginXy ori, int n, std::vector<bool> vec_isAlign){
	int y = n % ori.xBlock;
	int x = (n - y)/ori.yBlock;
	int isVer = 0;//���ʵ��ĸ�����ĸ���
	if(isVertex(ori,x,y))//���жϳ������ǲ��Ƕ���
		++isVer;
	bool get = false;//�ҵ�ת������
	int base = 0;
	while(isVer < 4){
		for(size_t i = 1; i <= base+1; ++i){
			++y;
			if(x > -1 && x < ori.yBlock && y > -1 && y <ori.xBlock){//����Ч����������
				if(vec_isAlign[x*ori.xBlock + y] == true){//�ҵ�
					return x * ori.xBlock + y;
				}else if(isVertex(ori,x,y)){//���ʵ����󶥵�
					++isVer;
				}
			}
		}
		for(size_t i = 1; i <= base+1; ++i){
			++x;
			if(x > -1 && x < ori.yBlock && y > -1 && y <ori.xBlock){//����Ч����������
				if(vec_isAlign[x*ori.xBlock + y] == true){//�ҵ�
					return x * ori.xBlock + y;
				}else if(isVertex(ori,x,y)){//���ʵ����󶥵�
					++isVer;
				}
			}
		}
		for(size_t i = 1; i <= base+2; ++i){
			--y;
			if(x > -1 && x < ori.yBlock && y > -1 && y <ori.xBlock){//����Ч����������
				if(vec_isAlign[x*ori.xBlock + y] == true){//�ҵ�
					return x * ori.xBlock + y;
				}else if(isVertex(ori,x,y)){//���ʵ����󶥵�
					++isVer;
				}
			}
		}
		for(size_t i = 1; i <= base+2; ++i){
			--x;
			if(x > -1 && x < ori.yBlock && y > -1 && y <ori.xBlock){//����Ч����������
				if(vec_isAlign[x*ori.xBlock + y] == true){//�ҵ�
					return x * ori.xBlock + y;
				}else if(isVertex(ori,x,y)){//���ʵ����󶥵�
					++isVer;
				}
			}
		}
		base += 2;
	}
	return n;//û���ҵ��򷵻ر��������
}

std::vector<Eigen::Matrix4f> vec_transform;//�洢��Ч�ı任����

void patchToPairAlign(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &p1_cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &p2_cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &tgtPatch, OriginXy oriXy){
	std::vector<bool> vec_isAlign(oriXy.xBlock*oriXy.yBlock, false);
	std::vector<bool> vec_isNull(oriXy.xBlock*oriXy.yBlock, false);
	Eigen::Matrix4f pairTransform = Eigen::Matrix4f::Identity();//�任����
	//���õ�λ�������任����
	for(size_t i = 0; i < oriXy.xBlock*oriXy.yBlock; ++i)
		vec_transform.push_back(pairTransform);
	for(size_t i = 0; i < oriXy.xBlock*oriXy.yBlock; ++i){
		std::cout << "��" << i+1 << "/" << oriXy.xBlock * oriXy.yBlock << "�Կ�ʼ" << std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTmp(new pcl::PointCloud<pcl::PointXYZ>);
		tgtPatch.push_back(cloudTmp);
		//showCloudsLeft(p1_cloud[i],p2_cloud[i]);
		if(p2_cloud[i]->width == 0){//Ŀ�����Ϊ�գ�����Ҫ����
			vec_isAlign[i] = false;
			vec_isNull[i] = true;
			std::cout << "��" << i+1 << "/" << oriXy.xBlock * oriXy.yBlock << "�Բ���Ҫ��׼" << std::endl << std::endl;
		}else if(p1_cloud[i]->width < 10 || p2_cloud[i]->width < 10){//��̫��ʱ��������src����ֻ��һ���㣩��align������������������Ҫ���ٸ�����У���ʱ��10�����ϡ�(kdtree_flann.hpp line136 ���е��Ʊʼ�)
			std::cout << "��" << i+1 << "/" << oriXy.xBlock * oriXy.yBlock << "����׼ʧ��" << std::endl << std::endl;
		}else{
			bool isAlign = pairAlign(p1_cloud[i], p2_cloud[i], tgtPatch[i], pairTransform, true);//������Ҫ���Խ����²���
			if(isAlign){
				vec_transform[i] = pairTransform;
				vec_isAlign[i] = true;
				std::cout << "��" << i+1 << "/" << oriXy.xBlock * oriXy.yBlock << "����׼���" << std::endl << std::endl;
			}else{
				std::cout << "��" << i+1 << "/" << oriXy.xBlock * oriXy.yBlock << "����׼ʧ��" << std::endl << std::endl;
			}
		}
	}
	//�����ʣ��û�������׼�ģ��������ڿռ���������ĵ��ƶԵ���Чת����������׼
	std::cout << "�ﲻ����׼�����ĵ��ƶԣ����������ת��������׼��" << std::endl;
	bool hasLeft = false;//�Ƿ���ʣ����Ҫ��׼��
	for(size_t i = 0; i < vec_isAlign.size(); ++i){
		if(vec_isAlign[i] == false && vec_isNull[i] == false){//û����׼�Ҳ�Ϊ�յĵ���
			hasLeft = true;
			pairTransform = vec_transform[getClosestTransformIdx(oriXy, i, vec_isAlign)];
			pcl::transformPointCloud(*p2_cloud[i], *tgtPatch[i], pairTransform);
			std::cout << "��" << i+1 << "/" << oriXy.xBlock * oriXy.yBlock << "����׼���" << std::endl << std::endl;
		}
	}
	if(!hasLeft)
		std::cout << "��" << std::endl;
}


//���˺ϲ�֮�����ı����Ϣ�����磺��С����ֵ�ȣ��⣬������Ϣ���ݵ�һ��ɨ��ͷ�ĵ�����д
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
	//����Point3D������ϵ
	//�����Ϊʲôlas�ļ����水��PointDataRecord������ϵ�洢�����꣬������ֵȴ�õ�Point3D������ϵ��
	// ͷ�ļ��������Ǿ�������ϵ��points������Ϣ�����ڽ�������꣨pointRecord����ԭ��������Ϣ���õľ�������ϵ��
	// ͷ�ļ��������Ǿ�������ϵ��points������Ϣ�����ڽ�������꣨pointRecord����ԭ��������Ϣ���õľ�������ϵ��
	// ���Բ�����ԭ����Ϣͷ�������Щ��Ϣ�����ã����磺��ֵ��
	// 
	// 
	//��һ��
	// Create PulicHeaderBlock
	PublicHeaderBlock publicHeader;
	//sprintf(publicHeader.fileSign,header1.fileSign);
	memcpy(publicHeader.fileSign, header1.fileSign, sizeof(publicHeader.fileSign));
	publicHeader.fileSourceID = header1.fileSourceID;
	publicHeader.reserved = header1.reserved;
	publicHeader.GUID1 = header1.GUID1;
	publicHeader.GUID2 = header1.GUID2;
	publicHeader.GUID3 = header1.GUID3;
	//sprintf((char *)publicHeader.GUID4, (const char*)header1.GUID4);//ת��������--------Ӱ�첻��ɣ�--------------------------
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
	publicHeader.varRecordNum = header1.varRecordNum + header2.varRecordNum;//�õ���header1,header2-------------------
	publicHeader.dataFormat = header1.dataFormat;
	publicHeader.pointRecordLen = sizeof(PointDataRecord);//-------��һ��ÿ��las�ļ�Ӧ����һ�µİɣ�--------------
	publicHeader.pointRecordNum = header1.pointRecordNum + header2.pointRecordNum;//�õ���header1,header2-------------------
	memcpy(publicHeader.returnPointNum, header1.returnPointNum, sizeof(publicHeader.returnPointNum));

	//�������ֵ��point3d����ϵ���������꣩�ģ�lo2�е���ֵ��replacePointData()�������Ѿ����¹����������ﲻ���ٱ���lo2��point3dȷ����ֵ�ˡ���lo1����ֵ��û�б�ġ�
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

	//�ڶ���
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
			std::cout << "lo1����д��" << i << std::endl;
		}
	}
	std::cout << std::endl << std::endl;
	// lo2 Point3Dת����PointRecord����ϵ
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
			std::cout << "lo2����д��" << i << std::endl;
		}
	}

	lasFile.close();
	std::cout << "las file(two-in-one) is OK" << std::endl;
	return true;
}


void saveSomePatchsAsLas(const char *file_name, CLasOperator &lo_somePatchs, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &tgtPatch, std::vector<int> const vec_idx, OriginXy const oriXy){
	std::vector<Point3DC> vec_point3dc;
	for(size_t n = 0; n < vec_idx.size(); ++n){
		//if(n > 4)break;//�������patch�Ͳ�Ҫ��
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
	std::cout << "��ʼ����patch" << std::endl;
	lo_somePatchs.savePoint3dcAsLas(file_name, &vec_point3dc);
	std::cout << "patch�������" << std::endl;
}

void rigidTransform(const char * file_name, const char * file_name_transform){
	CLasOperator lo;
	lo.readLasFile(file_name);
	std::vector<Point3DC> vec_point3dc = lo.getPoints();
	//û�м���ת
	//ƽ����Ϣ
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

	//������Ƭ����
	loadLasData(argc,argv,vec_lo);

	//�и���Ϣ
	OriginXy oriXy;
	int step = 50;//���ָ����Ϊ���ͷ��㣬�����������ת��--------------------
	std::cout << "�����п鲽��" << std::endl;
	std::cin >> step;
	std::cout << std::endl;
	getPatchInfo(vec_lo[0].getPublicHeader(), oriXy, step);
	
	//�ֿ��ĵ��ƴ洢��p1_cloud,p2_cloud������
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> p1_cloud, p2_cloud;
	std::vector<int> pointIdx1;//�洢ÿ�����ȥ�򣨷ֵ��ڼ�����ƣ�
	std::vector<int> pointIdx2;
	toPatchs(p1_cloud, vec_lo[0], oriXy, pointIdx1);	
	toPatchs(p2_cloud, vec_lo[1], oriXy, pointIdx2);

	//����ÿƬ���ƴ�С
	std::cout << std::endl;
	for(size_t i = 0; i < oriXy.xBlock*oriXy.yBlock; ++i){
		std::cout << i+1 << ":  " << p1_cloud[i]->width << "  " << p2_cloud[i]->width << std::endl;
	}

	//��Ӧ���ƶ���׼
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> tgtPatch;//�洢��׼���Ŀ�����
	patchToPairAlign(p1_cloud, p2_cloud, tgtPatch, oriXy);
	std::cout << std::endl << std::endl << "��׼���" << std::endl;

	//�鿴�任����
	std::vector<double> vec_dist;//����ƽ�Ʊ任����
	for(size_t i = 0; i < vec_transform.size(); ++i){
		std::cout << i+1 << std::endl;
		std::cout << vec_transform[i] << std::endl;
		vec_dist.push_back(std::pow(vec_transform[i](0,3),2) + std::pow(vec_transform[i](1,3),2) + std::pow(vec_transform[i](2,3),2));
	}
	//���������򣬰��Ӵ�С��ԭ��Ŵ���
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
	//���仯�̶Ƚϴ�ļ����ɵ��ƣ��̶�����ɫ����
	CLasOperator lo_somePatchs;
	saveSomePatchsAsLas("E:\\PCL\\PCLdata\\somePatchs_xa.las", lo_somePatchs, tgtPatch, vec_idx, oriXy);

	///////////////////////

	//��׼��lo2�еĲ�����Ϣ����,Ҫ���õ�����pointIdx2 ��������������������������
	std::cout << "����lo2�в�����Ϣ" << std::endl;
	vec_lo[1].replacePointData(tgtPatch, oriXy, pointIdx2);
	std::cout << "�������" << std::endl;

	tgtPatch.clear();//�ͷŵ�

	char c2 = getchar();
	std::cout << "������" << std::endl;
	char c1 = getchar();

	//��󽫵�һƬ���ƺ���׼��ĵڶ�Ƭ���� һ��д����̣��ϲ��󲿷���ϢҪ�����ģ�
	std::cout << "д�����" << std::endl;
	saveDoubleLasFile("E:\\PCL\\PCLdata\\two_in_one_xa.las", vec_lo[0], vec_lo[1]);

	stop = time(NULL);
	std::cout << std::endl << "��ʱ��  " << (stop - start) << std::endl;

	std::cout << "��¼" << std::endl;
	char c3 = getchar();
	char c4 = getchar();
	return 0;
}

/* ]--- */
