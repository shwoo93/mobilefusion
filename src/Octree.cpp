#include "Octree.h"

namespace MobileFusion {
    OctreeNode::OctreeNode (float x, float y, float z, float size_x, float size_y, float size_z)
        : ctr_x_ (x)
        , ctr_y_ (y)
        , ctr_z_ (z)
        , size_ (size_x)
        , d_ (-1) //distance
        , w_ (0) //weight
        , M_ (0)
        , nsample_ (0) {
        }
    OctreeNode::OctreeNode() {
    }

    OctreeNode::~OctreeNode() {
    }

    void OctreeNode::getCenter (float &x, float &y, float &z) const {
        x = ctr_x_;
        y = ctr_y_;
        z = ctr_z_;
    };

    void OctreeNode::getSize (float &size_x, float &size_y, float &size_z) const {
        size_x = size_;
        size_y = size_;
        size_z = size_;
    }

    float OctreeNode::getMaxSize () const {
        return (std::sqrt (3) * size_ );
    }

    float OctreeNode::getMinSize () const {
        return (size_);
    }

    bool OctreeNode::hasChildren () const {
        return (!children_.empty());
    }

    std::vector<OctreeNode::Ptr>& OctreeNode::getChildren() {
        return children_;
    }

    const std::vector<OctreeNode::Ptr>& OctreeNode::getChildren() const {
        return children_;
    }

    void OctreeNode::getLeaves (std::vector<OctreeNode::Ptr> &leaves, int num_levels) {
        for (size_t i = 0; i < children_.size(); i++) {
            const OctreeNode::Ptr &child = children_[i];
            if(child->hasChildren () && num_levels != 0)
                child->getLeaves (leaves, num_levels -1);
            else
                leaves.push_back (child);
        }
    }

    OctreeNode * OctreeNode::getContainingVoxel (float x, float y, float z, float min_size) {
        if (!hasChildren () || (min_size > 0 && size_ <= min_size))
            return (this);
        else {
            return children_[((x-ctr_x_) > 0) * 4 + ((y-ctr_y_)) * 2 + (z-ctr_z_ > 0) ]->getContainingVoxel (x, y, z, min_size);
        }
    }

    bool OctreeNode::getData (float &d, float &w) const {
        d = d_;
        w = w_;
        return (true);
    }

    bool OctreeNode::setData (float d, float w) {
        d_ = d;
        w_ = w;
        return (true);
    }

    bool OctreeNode::addObservation (float d_new, float w_new, float max_weight) {
        float d_old = d_;
        d_ = (d_ * w_ + d_new * w_new) / (w_ + w_new);
        w_ += w_new;
        if (w_ > max_weight)
            w_ = max_weight;
        M_ += w_new * (d_new - d_) * (d_new - d_old);
        ++nsample_;
        return (true);
    }

    bool OctreeNode::addObservation (float d_new, float  w_new, float max_weight,
            uint8_t r, uint8_t g, uint8_t b) {
        return (addObservation (d_new, w_new, max_weight));
    }

    bool OctreeNode::getRGB (uint8_t &r, uint8_t &g, uint8_t &b) const {
        r = g = b = 127;
        return (false);
    }

    OctreeNode* OctreeNode::instantiateNode (float x, float y, float z, float sx, float sy, float sz) {
        return (new OctreeNode (x, y, z, sx, sy, sz));
    }

    std::string OctreeNode::getTypeString () {
        return ("NOCOLOR");
    }

    OctreeNode* OctreeNode::instantiateByTypeString (const std::string &str) {
        if (str == "NOCOLOR")
            return (new OctreeNode);
        else if (str == "RGB")
            return (new RGBNode);
        else if (str == "RGBNorMalized")
            return (new RGBNormalized);
        //PCL_ERROR ("[OctreeNode::instantiateByTypeString] Requested invalid type string &s\n", str.c_str ());
        else
            return (NULL);
    }

    OctreeNode* OctreeNode::instantiateByTypeString (const std::string &str,
                                float x, float y, float z, float sx, float sy, float sz) {
        OctreeNode* empty_node = instantiateByTypeString (str);
        OctreeNode* node = empty_node->instantiateNode (x, y, z, sx, sy, sz);
        delete empty_node;
        return (node);
    }

    void OctreeNode::updateAverage () {
        if (children_.empty())
            return;

        float d_avg = 0;
        float w_avg = 0;
        int ngood = 0;
        for (size_t i = 0 ; i < children_.size() ; ++i) {
            children_[i]->updateAverage();
            if (children_[i]->w_ > 0) {
                d_avg += children_[i]->d_;
                w_avg += children_[i]->w_;
                ++ngood;
            }
        }
        if (ngood>0) {
            d_ = d_avg/ngood;
            w_ = w_avg/ngood;
        }
    }

    std::vector<OctreeNode::Ptr>& OctreeNode::split () {
        children_.resize (8);
        float off_x = size_ / 4;
        float off_y = size_ / 4;
        float off_z = size_ / 4;
        float newsize_x = size_ / 2;
        float newsize_y = size_ / 2;
        float newsize_z = size_ / 2;
        children_[0].reset (instantiateNode (ctr_x_-off_x, ctr_y_-off_y, ctr_z_-off_z, newsize_x, newsize_y, newsize_z));
        children_[1].reset (instantiateNode (ctr_x_-off_x, ctr_y_-off_y, ctr_z_+off_z, newsize_x, newsize_y, newsize_z));
        children_[2].reset (instantiateNode (ctr_x_-off_x, ctr_y_+off_y, ctr_z_-off_z, newsize_x, newsize_y, newsize_z));
        children_[3].reset (instantiateNode (ctr_x_-off_x, ctr_y_+off_y, ctr_z_+off_z, newsize_x, newsize_y, newsize_z));
        children_[4].reset (instantiateNode (ctr_x_+off_x, ctr_y_-off_y, ctr_z_-off_z, newsize_x, newsize_y, newsize_z));
        children_[5].reset (instantiateNode (ctr_x_+off_x, ctr_y_-off_y, ctr_z_+off_z, newsize_x, newsize_y, newsize_z));
        children_[6].reset (instantiateNode (ctr_x_+off_x, ctr_y_+off_y, ctr_z_-off_z, newsize_x, newsize_y, newsize_z));
        children_[7].reset (instantiateNode (ctr_x_+off_x, ctr_y_+off_y, ctr_z_+off_z, newsize_x, newsize_y, newsize_z));
        return (children_);
    }

    void OctreeNode::splitRecursive(int num_left) {
        if (num_left <= 0)
            return;
        split();
        for (size_t i = 0 ; i < children_.size() ; ++i) {
            children_[i]->splitRecursive (num_left-1);
        }
    }

    float OctreeNode::getVariance () const {
        if (nsample_ < 5)
            return (std::numeric_limits<float>::infinity ());
        return ((M_/w_)*(nsample_/(nsample_-1)));
    }

    //RGBNode
    RGBNode::RGBNode(float x, float y, float z, float size_x, float size_y, float size_z)
    : r_(0)
    , g_(0)
    , b_(0)
    , OctreeNode (x, y, z, size_x, size_y, size_z) {
    }

    RGBNode::RGBNode()
    : OctreeNode() {
    }

    RGBNode::~RGBNode() {
    }

    bool RGBNode::addObservation (float d_new, float w_new, float max_weight,
            uint8_t r, uint8_t g, uint8_t b) {
        float wsum = w_ + w_new;
        r_ = static_cast<uint8_t>( (w_*r_ + w_new*r) / wsum);
        g_ = static_cast<uint8_t>( (w_*g_ + w_new*g) / wsum);
        b_ = static_cast<uint8_t>( (w_*b_ + w_new*b) / wsum);
        return (OctreeNode::addObservation (d_new, w_new, max_weight));
    }

    bool RGBNode::getRGB (uint8_t &r, uint8_t &g, uint8_t &b) const {
        r = r_;
        g = g_;
        b = b_;
        return (true);
    }

    OctreeNode* RGBNode::instantiateNode (float x, float y, float z, float sx, float sy, float sz) {
        return (new RGBNode (x, y, z, sx, sy, sz));
    }

    std::string RGBNode::getTypeString () {
        return ("RGB");
    }

    //RGBNormalized
    RGBNormalized::RGBNormalized (float x, float y, float z, float size_x, float size_y, float size_z)
    : r_n_ (0)
    , g_n_ (0)
    , b_n_ (0)
    , i_ (0)
    , OctreeNode (x, y, z, size_x, size_y, size_z) {
    }

    RGBNormalized::RGBNormalized()
    : OctreeNode() {
    }

    RGBNormalized::~RGBNormalized() {
    }

    bool RGBNormalized::addObservation (float d_new, float w_new, float max_weight,
            uint8_t r, uint8_t g, uint8_t b) {
        float wsum = w_ + w_new;
        float i = std::sqrt ((float)r * (float)r + (float)g * (float)g + (float)b * (float)b);
        float r_f = r / i;
        float g_f = g / i;
        float b_f = b / i;
        r_n_ = (w_*r_n_ + w_new*r_f) / wsum;
        g_n_ = (w_*g_n_ + w_new*g_f) / wsum;
        b_n_ = (w_*b_n_ + w_new*b_f) / wsum;
        i_ = (w_*i_ + w_new*i) / wsum;
        return (OctreeNode::addObservation (d_new, w_new, max_weight));
    }

    bool RGBNormalized::getRGB (uint8_t &r, uint8_t &g, uint8_t &b) const {
        r = r_n_ * i_;
        g = g_n_ * i_;
        b = b_n_ * i_;
        return (true);
    }

    OctreeNode* RGBNormalized::instantiateNode (float x, float y, float z, float sx, float sy, float sz) {
        return (new RGBNormalized (x, y, z, sx, sy, sz));
    }

    std::string RGBNormalized::getTypeString () {
        return ("RGBNormalized");
    }

    //Octree
    Octree::Octree (size_t res_x, size_t res_y, size_t res_z, float size_x, float size_y, float size_z, const std::string voxel_type)
    : res_x_ (res_x)
    , res_y_ (res_y)
    , res_z_ (res_z)
    , size_x_ (size_x)
    , size_y_ (size_y)
    , size_z_ (size_z)
    , voxel_type_ (voxel_type) {
    }

    Octree::Octree () {
    }

    void Octree::init (int num_splits) {
        root_.reset (OctreeNode::instantiateByTypeString
                (voxel_type_, 0, 0, 0, size_x_, size_y_, size_z_));
        root_->splitRecursive (num_splits);
    }

    void Octree::init (float max_size_x, float max_size_y, float max_size_z) {
        int desired_res = std::max (size_x_/max_size_x, std::max (size_y_ / max_size_y, size_z_/max_size_z));
        int num_levels = std::ceil (std::log (desired_res) / std::log(2));
        init (num_levels);
    }

    OctreeNode::Ptr& Octree::getRoot() {
        return root_;
    }

    void Octree::getLeaves (std::vector<OctreeNode::Ptr> &leaves, int num_levels) const {
        //pcl::console::TicToc tt;
        //tt.tic();
        if (num_levels == 0)
            leaves.push_back (root_);
        else
            root_->getLeaves (leaves, num_levels-1);
    }

    void Octree::getLeaves (std::vector<OctreeNode::Ptr> &leaves, float max_size_x, float max_size_y, float max_size_z) const {
        int desired_res = std::max (size_x_/max_size_x, std::max (size_y_ / max_size_y, size_z_/max_size_z));
        int num_levels = std::ceil (std::log (desired_res) / std::log (2));
        getLeaves (leaves, num_levels);
    }

    const OctreeNode * Octree::getContainingVoxel (float x, float y, float z, float min_size) const {
        if (pcl_isnan (z) || std::fabs (x) > size_x_/2 || std::fabs(y) > size_y_/2 || std::fabs(z) > size_z_/2)
            return NULL;
        return (root_->getContainingVoxel (x, y, z, min_size));
    }

    OctreeNode * Octree::getContainingVoxel (float x, float y, float z, float min_size) {
        if (pcl_isnan (z) || std::fabs (x) > size_x_/2 || std::fabs(y) > size_y_/2 || std::fabs(z) > size_z_/2)
            return NULL;
        return (root_->getContainingVoxel (x, y, z, min_size));
    }
}






















