#include "gui.hpp"
#include "../app.hpp"

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace kiln {
namespace {

#define VK_CHECK(expr)                                                                 \
    do {                                                                               \
        VkResult _r = (expr);                                                          \
        if (_r != VK_SUCCESS) throw std::runtime_error("vulkan error " + std::to_string(_r)); \
    } while (0)

struct VertexGPU {
    float p[3];
    float n[3];
};

struct PushPC {
    float model[16];
    float color[4];
};

struct FrameUBO {
    float viewproj[16];
    float sun[4];
};

struct DrawItem {
    uint32_t first;
    uint32_t count;
    Mat4 model;
    Vec3 color;
};

struct VK {
    GLFWwindow* window = nullptr;
    VkInstance instance = VK_NULL_HANDLE;
    VkSurfaceKHR surface = VK_NULL_HANDLE;
    VkPhysicalDevice gpu = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    VkQueue queue = VK_NULL_HANDLE;
    uint32_t qfamily = 0;
    VkSwapchainKHR swapchain = VK_NULL_HANDLE;
    VkFormat swap_format = VK_FORMAT_B8G8R8A8_UNORM;
    VkExtent2D extent{1280, 800};
    std::vector<VkImage> images;
    std::vector<VkImageView> views;
    std::vector<VkFramebuffer> fbs;
    VkImage depth = VK_NULL_HANDLE;
    VkDeviceMemory depth_mem = VK_NULL_HANDLE;
    VkImageView depth_view = VK_NULL_HANDLE;
    VkRenderPass rp = VK_NULL_HANDLE;
    VkCommandPool pool = VK_NULL_HANDLE;
    VkDescriptorSetLayout dsl = VK_NULL_HANDLE;
    VkPipelineLayout pl = VK_NULL_HANDLE;
    VkPipeline pipeline = VK_NULL_HANDLE;
    VkDescriptorPool dp = VK_NULL_HANDLE;
    VkDescriptorPool imgui_dp = VK_NULL_HANDLE;
    VkBuffer vbo = VK_NULL_HANDLE;
    VkDeviceMemory vbo_mem = VK_NULL_HANDLE;
    VkBuffer ubo = VK_NULL_HANDLE;
    VkDeviceMemory ubo_mem = VK_NULL_HANDLE;
    void* ubo_map = nullptr;
    VkDescriptorSet dset = VK_NULL_HANDLE;
    VkShaderModule vs = VK_NULL_HANDLE;
    VkShaderModule fs = VK_NULL_HANDLE;
    static constexpr int kMaxFrames = 2;
    VkCommandBuffer cmds[kMaxFrames]{};
    VkSemaphore img_avail[kMaxFrames]{};
    VkSemaphore render_done[kMaxFrames]{};
    VkFence fences[kMaxFrames]{};
    int frame = 0;
    std::vector<DrawItem> draws;
};

uint32_t find_memory(VkPhysicalDevice gpu, uint32_t bits, VkMemoryPropertyFlags flags) {
    VkPhysicalDeviceMemoryProperties mp;
    vkGetPhysicalDeviceMemoryProperties(gpu, &mp);
    for (uint32_t i = 0; i < mp.memoryTypeCount; ++i) {
        if ((bits & (1u << i)) && (mp.memoryTypes[i].propertyFlags & flags) == flags) return i;
    }
    throw std::runtime_error("no memory type");
}

VkShaderModule load_shader(VkDevice dev, const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("missing shader " + path);
    std::vector<char> code((std::istreambuf_iterator<char>(f)), {});
    if (code.size() % 4) throw std::runtime_error("bad spirv " + path);
    VkShaderModuleCreateInfo ci{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    ci.codeSize = code.size();
    ci.pCode = reinterpret_cast<const uint32_t*>(code.data());
    VkShaderModule m;
    VK_CHECK(vkCreateShaderModule(dev, &ci, nullptr, &m));
    return m;
}

void create_buffer(VK& vk, VkDeviceSize size, VkBufferUsageFlags usage, VkBuffer& buf, VkDeviceMemory& mem,
                   void** map) {
    VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bi.size = size;
    bi.usage = usage;
    bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    VK_CHECK(vkCreateBuffer(vk.device, &bi, nullptr, &buf));
    VkMemoryRequirements req;
    vkGetBufferMemoryRequirements(vk.device, buf, &req);
    VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    ai.allocationSize = req.size;
    ai.memoryTypeIndex = find_memory(vk.gpu, req.memoryTypeBits,
                                     VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    VK_CHECK(vkAllocateMemory(vk.device, &ai, nullptr, &mem));
    VK_CHECK(vkBindBufferMemory(vk.device, buf, mem, 0));
    if (map) VK_CHECK(vkMapMemory(vk.device, mem, 0, size, 0, map));
}

std::string shader_path(const char* name) {
    const char* roots[] = {"shaders/", "../shaders/", "build/shaders/", "/usr/local/share/kiln/shaders/"};
    for (const char* r : roots) {
        std::string p = std::string(r) + name;
        std::ifstream f(p);
        if (f) return p;
    }
    return std::string("shaders/") + name;
}

void create_swapchain(VK& vk) {
    VkSurfaceCapabilitiesKHR caps;
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(vk.gpu, vk.surface, &caps);
    uint32_t nfmt = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(vk.gpu, vk.surface, &nfmt, nullptr);
    std::vector<VkSurfaceFormatKHR> fmts(nfmt);
    vkGetPhysicalDeviceSurfaceFormatsKHR(vk.gpu, vk.surface, &nfmt, fmts.data());
    vk.swap_format = fmts[0].format;
    for (auto& f : fmts) {
        if (f.format == VK_FORMAT_B8G8R8A8_UNORM || f.format == VK_FORMAT_B8G8R8A8_SRGB) {
            vk.swap_format = f.format;
            break;
        }
    }
    int w, h;
    glfwGetFramebufferSize(vk.window, &w, &h);
    vk.extent = caps.currentExtent;
    if (vk.extent.width == 0xFFFFFFFFu) {
        vk.extent.width = std::clamp(uint32_t(w), caps.minImageExtent.width, caps.maxImageExtent.width);
        vk.extent.height = std::clamp(uint32_t(h), caps.minImageExtent.height, caps.maxImageExtent.height);
    }
    uint32_t img_count = caps.minImageCount + 1;
    if (caps.maxImageCount && img_count > caps.maxImageCount) img_count = caps.maxImageCount;
    VkSwapchainCreateInfoKHR ci{VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
    ci.surface = vk.surface;
    ci.minImageCount = img_count;
    ci.imageFormat = vk.swap_format;
    ci.imageColorSpace = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
    ci.imageExtent = vk.extent;
    ci.imageArrayLayers = 1;
    ci.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    ci.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    ci.preTransform = caps.currentTransform;
    ci.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    ci.presentMode = VK_PRESENT_MODE_FIFO_KHR;
    ci.clipped = VK_TRUE;
    VK_CHECK(vkCreateSwapchainKHR(vk.device, &ci, nullptr, &vk.swapchain));
    uint32_t n = 0;
    vkGetSwapchainImagesKHR(vk.device, vk.swapchain, &n, nullptr);
    vk.images.resize(n);
    vkGetSwapchainImagesKHR(vk.device, vk.swapchain, &n, vk.images.data());
    vk.views.resize(n);
    for (uint32_t i = 0; i < n; ++i) {
        VkImageViewCreateInfo vi{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        vi.image = vk.images[i];
        vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
        vi.format = vk.swap_format;
        vi.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        VK_CHECK(vkCreateImageView(vk.device, &vi, nullptr, &vk.views[i]));
    }
}

void create_depth(VK& vk) {
    VkImageCreateInfo ii{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    ii.imageType = VK_IMAGE_TYPE_2D;
    ii.format = VK_FORMAT_D32_SFLOAT;
    ii.extent = {vk.extent.width, vk.extent.height, 1};
    ii.mipLevels = 1;
    ii.arrayLayers = 1;
    ii.samples = VK_SAMPLE_COUNT_1_BIT;
    ii.tiling = VK_IMAGE_TILING_OPTIMAL;
    ii.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    VK_CHECK(vkCreateImage(vk.device, &ii, nullptr, &vk.depth));
    VkMemoryRequirements req;
    vkGetImageMemoryRequirements(vk.device, vk.depth, &req);
    VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    ai.allocationSize = req.size;
    ai.memoryTypeIndex = find_memory(vk.gpu, req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    VK_CHECK(vkAllocateMemory(vk.device, &ai, nullptr, &vk.depth_mem));
    vkBindImageMemory(vk.device, vk.depth, vk.depth_mem, 0);
    VkImageViewCreateInfo vi{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    vi.image = vk.depth;
    vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
    vi.format = VK_FORMAT_D32_SFLOAT;
    vi.subresourceRange = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1};
    VK_CHECK(vkCreateImageView(vk.device, &vi, nullptr, &vk.depth_view));
}

void create_framebuffers(VK& vk) {
    vk.fbs.resize(vk.views.size());
    for (size_t i = 0; i < vk.views.size(); ++i) {
        VkImageView att[2] = {vk.views[i], vk.depth_view};
        VkFramebufferCreateInfo fi{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
        fi.renderPass = vk.rp;
        fi.attachmentCount = 2;
        fi.pAttachments = att;
        fi.width = vk.extent.width;
        fi.height = vk.extent.height;
        fi.layers = 1;
        VK_CHECK(vkCreateFramebuffer(vk.device, &fi, nullptr, &vk.fbs[i]));
    }
}

void destroy_swapchain_objects(VK& vk) {
    for (auto fb : vk.fbs) vkDestroyFramebuffer(vk.device, fb, nullptr);
    vk.fbs.clear();
    if (vk.depth_view) vkDestroyImageView(vk.device, vk.depth_view, nullptr);
    if (vk.depth) vkDestroyImage(vk.device, vk.depth, nullptr);
    if (vk.depth_mem) vkFreeMemory(vk.device, vk.depth_mem, nullptr);
    vk.depth_view = VK_NULL_HANDLE;
    vk.depth = VK_NULL_HANDLE;
    vk.depth_mem = VK_NULL_HANDLE;
    for (auto v : vk.views) vkDestroyImageView(vk.device, v, nullptr);
    vk.views.clear();
    if (vk.swapchain) vkDestroySwapchainKHR(vk.device, vk.swapchain, nullptr);
    vk.swapchain = VK_NULL_HANDLE;
}

void recapture_swapchain(VK& vk) {
    vkDeviceWaitIdle(vk.device);
    destroy_swapchain_objects(vk);
    create_swapchain(vk);
    create_depth(vk);
    create_framebuffers(vk);
}

void upload_scene(VK& vk, const Scene& scene) {
    std::vector<VertexGPU> verts;
    vk.draws.clear();
    for (int ni = 0; ni < static_cast<int>(scene.nodes.size()); ++ni) {
        const Node& n = scene.nodes[ni];
        if (n.type != NodeType::Mesh || n.mesh < 0 || n.mesh >= static_cast<int>(scene.meshes.size())) continue;
        const Mesh& mesh = scene.meshes[n.mesh];
        DrawItem d;
        d.first = static_cast<uint32_t>(verts.size());
        d.count = static_cast<uint32_t>(mesh.indices.size());
        d.model = scene.world_matrix(ni);
        d.color = {0.72f, 0.73f, 0.76f};
        if (n.material >= 0 && n.material < static_cast<int>(scene.materials.size()))
            d.color = scene.materials[n.material].albedo;
        for (uint32_t ix : mesh.indices) {
            const Vertex& v = mesh.vertices[ix];
            verts.push_back({{v.p.x, v.p.y, v.p.z}, {v.n.x, v.n.y, v.n.z}});
        }
        vk.draws.push_back(d);
    }
    if (vk.vbo) {
        vkDestroyBuffer(vk.device, vk.vbo, nullptr);
        vkFreeMemory(vk.device, vk.vbo_mem, nullptr);
        vk.vbo = VK_NULL_HANDLE;
    }
    if (verts.empty()) return;
    void* map = nullptr;
    create_buffer(vk, verts.size() * sizeof(VertexGPU), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, vk.vbo, vk.vbo_mem, &map);
    std::memcpy(map, verts.data(), verts.size() * sizeof(VertexGPU));
    vkUnmapMemory(vk.device, vk.vbo_mem);
}

void copy_mat(float* dst, const Mat4& m) { std::memcpy(dst, m.m, 16 * sizeof(float)); }

}  // namespace

int run_gui(Session& session) {
    static char prompt_buf[1024] = {};
    std::vector<std::string> log;
    log.emplace_back("kiln ready. Try: add a red cube");
    bool scene_dirty = true;

    if (!glfwInit()) {
        std::cerr << "glfw init failed\n";
        return 1;
    }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
    GLFWwindow* win = glfwCreateWindow(1440, 900, "kiln", nullptr, nullptr);
    if (!win) {
        std::cerr << "glfw window failed\n";
        return 1;
    }

    VK vk;
    vk.window = win;
    try {
        VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
        app.pApplicationName = "kiln";
        app.apiVersion = VK_API_VERSION_1_2;
        uint32_t ext_count = 0;
        const char** ext = glfwGetRequiredInstanceExtensions(&ext_count);
        std::vector<const char*> exts(ext, ext + ext_count);
        VkInstanceCreateInfo ici{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
        ici.pApplicationInfo = &app;
        ici.enabledExtensionCount = ext_count;
        ici.ppEnabledExtensionNames = exts.data();
        VK_CHECK(vkCreateInstance(&ici, nullptr, &vk.instance));
        VK_CHECK(glfwCreateWindowSurface(vk.instance, win, nullptr, &vk.surface));

        uint32_t n = 0;
        vkEnumeratePhysicalDevices(vk.instance, &n, nullptr);
        std::vector<VkPhysicalDevice> gpus(n);
        vkEnumeratePhysicalDevices(vk.instance, &n, gpus.data());
        if (gpus.empty()) throw std::runtime_error("no GPU");
        vk.gpu = gpus[0];
        uint32_t nq = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(vk.gpu, &nq, nullptr);
        std::vector<VkQueueFamilyProperties> qps(nq);
        vkGetPhysicalDeviceQueueFamilyProperties(vk.gpu, &nq, qps.data());
        vk.qfamily = 0;
        for (uint32_t i = 0; i < nq; ++i) {
            VkBool32 present = false;
            vkGetPhysicalDeviceSurfaceSupportKHR(vk.gpu, i, vk.surface, &present);
            if ((qps[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) && present) {
                vk.qfamily = i;
                break;
            }
        }
        float prio = 1;
        VkDeviceQueueCreateInfo qci{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        qci.queueFamilyIndex = vk.qfamily;
        qci.queueCount = 1;
        qci.pQueuePriorities = &prio;
        const char* dexts[] = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};
        VkDeviceCreateInfo dci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
        dci.queueCreateInfoCount = 1;
        dci.pQueueCreateInfos = &qci;
        dci.enabledExtensionCount = 1;
        dci.ppEnabledExtensionNames = dexts;
        VK_CHECK(vkCreateDevice(vk.gpu, &dci, nullptr, &vk.device));
        vkGetDeviceQueue(vk.device, vk.qfamily, 0, &vk.queue);

        create_swapchain(vk);
        create_depth(vk);

        VkAttachmentDescription atts[2]{};
        atts[0].format = vk.swap_format;
        atts[0].samples = VK_SAMPLE_COUNT_1_BIT;
        atts[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        atts[0].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        atts[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        atts[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        atts[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        atts[0].finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
        atts[1].format = VK_FORMAT_D32_SFLOAT;
        atts[1].samples = VK_SAMPLE_COUNT_1_BIT;
        atts[1].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        atts[1].storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        atts[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        atts[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        atts[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        atts[1].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
        VkAttachmentReference color_ref{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
        VkAttachmentReference depth_ref{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
        VkSubpassDescription sub{};
        sub.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        sub.colorAttachmentCount = 1;
        sub.pColorAttachments = &color_ref;
        sub.pDepthStencilAttachment = &depth_ref;
        VkSubpassDependency dep{};
        dep.srcSubpass = VK_SUBPASS_EXTERNAL;
        dep.dstSubpass = 0;
        dep.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
        dep.dstStageMask = dep.srcStageMask;
        dep.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
        VkRenderPassCreateInfo rpci{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
        rpci.attachmentCount = 2;
        rpci.pAttachments = atts;
        rpci.subpassCount = 1;
        rpci.pSubpasses = &sub;
        rpci.dependencyCount = 1;
        rpci.pDependencies = &dep;
        VK_CHECK(vkCreateRenderPass(vk.device, &rpci, nullptr, &vk.rp));
        create_framebuffers(vk);

        VkDescriptorSetLayoutBinding b{};
        b.binding = 0;
        b.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        b.descriptorCount = 1;
        b.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
        VkDescriptorSetLayoutCreateInfo dl{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        dl.bindingCount = 1;
        dl.pBindings = &b;
        VK_CHECK(vkCreateDescriptorSetLayout(vk.device, &dl, nullptr, &vk.dsl));
        VkPushConstantRange pcr{VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PushPC)};
        VkPipelineLayoutCreateInfo plc{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        plc.setLayoutCount = 1;
        plc.pSetLayouts = &vk.dsl;
        plc.pushConstantRangeCount = 1;
        plc.pPushConstantRanges = &pcr;
        VK_CHECK(vkCreatePipelineLayout(vk.device, &plc, nullptr, &vk.pl));

        vk.vs = load_shader(vk.device, shader_path("mesh.vert.spv"));
        vk.fs = load_shader(vk.device, shader_path("mesh.frag.spv"));
        VkPipelineShaderStageCreateInfo stages[2]{};
        stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
        stages[0].module = vk.vs;
        stages[0].pName = "main";
        stages[1] = stages[0];
        stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        stages[1].module = vk.fs;

        VkVertexInputBindingDescription bind{0, sizeof(VertexGPU), VK_VERTEX_INPUT_RATE_VERTEX};
        VkVertexInputAttributeDescription attrs[2] = {
            {0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},
            {1, 0, VK_FORMAT_R32G32B32_SFLOAT, 12},
        };
        VkPipelineVertexInputStateCreateInfo vi{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
        vi.vertexBindingDescriptionCount = 1;
        vi.pVertexBindingDescriptions = &bind;
        vi.vertexAttributeDescriptionCount = 2;
        vi.pVertexAttributeDescriptions = attrs;
        VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
        ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        VkPipelineViewportStateCreateInfo vp{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
        vp.viewportCount = 1;
        vp.scissorCount = 1;
        VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
        rs.polygonMode = VK_POLYGON_MODE_FILL;
        rs.cullMode = VK_CULL_MODE_BACK_BIT;
        rs.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        rs.lineWidth = 1;
        VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
        ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        VkPipelineDepthStencilStateCreateInfo ds{VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
        ds.depthTestEnable = VK_TRUE;
        ds.depthWriteEnable = VK_TRUE;
        ds.depthCompareOp = VK_COMPARE_OP_LESS;
        VkPipelineColorBlendAttachmentState blend{};
        blend.colorWriteMask = 0xF;
        VkPipelineColorBlendStateCreateInfo cb{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
        cb.attachmentCount = 1;
        cb.pAttachments = &blend;
        VkDynamicState dyns[] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
        VkPipelineDynamicStateCreateInfo dyn{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
        dyn.dynamicStateCount = 2;
        dyn.pDynamicStates = dyns;
        VkGraphicsPipelineCreateInfo gpi{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
        gpi.stageCount = 2;
        gpi.pStages = stages;
        gpi.pVertexInputState = &vi;
        gpi.pInputAssemblyState = &ia;
        gpi.pViewportState = &vp;
        gpi.pRasterizationState = &rs;
        gpi.pMultisampleState = &ms;
        gpi.pDepthStencilState = &ds;
        gpi.pColorBlendState = &cb;
        gpi.pDynamicState = &dyn;
        gpi.layout = vk.pl;
        gpi.renderPass = vk.rp;
        VK_CHECK(vkCreateGraphicsPipelines(vk.device, VK_NULL_HANDLE, 1, &gpi, nullptr, &vk.pipeline));

        VkDescriptorPoolSize psz{VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 4};
        VkDescriptorPoolCreateInfo dpi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        dpi.maxSets = 4;
        dpi.poolSizeCount = 1;
        dpi.pPoolSizes = &psz;
        VK_CHECK(vkCreateDescriptorPool(vk.device, &dpi, nullptr, &vk.dp));
        create_buffer(vk, sizeof(FrameUBO), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, vk.ubo, vk.ubo_mem, &vk.ubo_map);
        VkDescriptorSetAllocateInfo dai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        dai.descriptorPool = vk.dp;
        dai.descriptorSetCount = 1;
        dai.pSetLayouts = &vk.dsl;
        VK_CHECK(vkAllocateDescriptorSets(vk.device, &dai, &vk.dset));
        VkDescriptorBufferInfo dbi{vk.ubo, 0, sizeof(FrameUBO)};
        VkWriteDescriptorSet wr{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        wr.dstSet = vk.dset;
        wr.descriptorCount = 1;
        wr.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        wr.pBufferInfo = &dbi;
        vkUpdateDescriptorSets(vk.device, 1, &wr, 0, nullptr);

        VkCommandPoolCreateInfo cpi{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        cpi.queueFamilyIndex = vk.qfamily;
        cpi.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        VK_CHECK(vkCreateCommandPool(vk.device, &cpi, nullptr, &vk.pool));
        VkCommandBufferAllocateInfo cai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        cai.commandPool = vk.pool;
        cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        cai.commandBufferCount = VK::kMaxFrames;
        VK_CHECK(vkAllocateCommandBuffers(vk.device, &cai, vk.cmds));
        for (int i = 0; i < VK::kMaxFrames; ++i) {
            VkSemaphoreCreateInfo si{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
            VkFenceCreateInfo fi{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
            fi.flags = VK_FENCE_CREATE_SIGNALED_BIT;
            VK_CHECK(vkCreateSemaphore(vk.device, &si, nullptr, &vk.img_avail[i]));
            VK_CHECK(vkCreateSemaphore(vk.device, &si, nullptr, &vk.render_done[i]));
            VK_CHECK(vkCreateFence(vk.device, &fi, nullptr, &vk.fences[i]));
        }

        VkDescriptorPoolSize ipools[] = {
            {VK_DESCRIPTOR_TYPE_SAMPLER, 1000},
            {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000},
            {VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000},
            {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000},
            {VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000},
            {VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000},
            {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000},
            {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000},
            {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000},
            {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000},
            {VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000},
        };
        VkDescriptorPoolCreateInfo idp{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        idp.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
        idp.maxSets = 1000 * IM_ARRAYSIZE(ipools);
        idp.poolSizeCount = IM_ARRAYSIZE(ipools);
        idp.pPoolSizes = ipools;
        VK_CHECK(vkCreateDescriptorPool(vk.device, &idp, nullptr, &vk.imgui_dp));

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGui::StyleColorsDark();
        ImGuiStyle& st = ImGui::GetStyle();
        st.WindowRounding = 6;
        st.Colors[ImGuiCol_WindowBg] = ImVec4(0.07f, 0.07f, 0.08f, 0.92f);
        st.Colors[ImGuiCol_Button] = ImVec4(0.88f, 0.42f, 0.18f, 1);
        ImGui_ImplGlfw_InitForVulkan(win, true);
        ImGui_ImplVulkan_InitInfo ii{};
        ii.Instance = vk.instance;
        ii.PhysicalDevice = vk.gpu;
        ii.Device = vk.device;
        ii.QueueFamily = vk.qfamily;
        ii.Queue = vk.queue;
        ii.DescriptorPool = vk.imgui_dp;
        ii.MinImageCount = 2;
        ii.ImageCount = static_cast<uint32_t>(vk.images.size());
        ii.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
        ii.RenderPass = vk.rp;
        ImGui_ImplVulkan_Init(&ii);

        double last_mx = 0, last_my = 0;
        glfwGetCursorPos(win, &last_mx, &last_my);

        while (!glfwWindowShouldClose(win) && session.running) {
            glfwPollEvents();
            if (scene_dirty) {
                upload_scene(vk, session.doc.scene);
                scene_dirty = false;
            }

            ImGui_ImplVulkan_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            ImGuiIO& io = ImGui::GetIO();
            ImGui::SetNextWindowPos({io.DisplaySize.x - 340, 0});
            ImGui::SetNextWindowSize({340, io.DisplaySize.y - 88});
            ImGui::Begin("agent", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
            ImGui::Text("kiln");
            ImGui::SameLine();
            ImGui::TextDisabled(" · Linux modeler");
            // Same session value the CLI flag writes, so --samples and the GUI
            // cannot disagree about the quality the renderer is running at.
            ImGui::SameLine();
            ImGui::TextDisabled(" · %dx%d samples", session.samples, session.samples);
            // The two look dials, read straight from the Session fields the CLI
            // flags write, so the readout cannot claim a pass the renderer is
            // not running.
            ImGui::TextDisabled("shading: shadow %d/%d, ao %d/%d", session.contact_shadow,
                                kContactShadowMax, session.ao, kAoMax);
            // Whether the next export bakes a thumbnail, read from the same
            // Session fields --glb-preview / --preview-size write.
            ImGui::TextDisabled("glb preview %s at %dx%d", session.glb_preview ? "on" : "off",
                                session.glb_preview_w, session.glb_preview_h);
            ImGui::Separator();
            ImGui::BeginChild("log", {0, -4}, true);
            for (auto& line : log) ImGui::TextWrapped("%s", line.c_str());
            ImGui::EndChild();
            ImGui::End();

            ImGui::SetNextWindowPos({0, 0});
            ImGui::SetNextWindowSize({200, io.DisplaySize.y - 88});
            ImGui::Begin("scene", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
            for (int i = 0; i < static_cast<int>(session.doc.scene.nodes.size()); ++i) {
                if (ImGui::Selectable(session.doc.scene.nodes[i].name.c_str(), session.doc.scene.selection == i))
                    session.doc.scene.selection = i;
            }
            ImGui::End();

            ImGui::SetNextWindowPos({0, io.DisplaySize.y - 88});
            ImGui::SetNextWindowSize({io.DisplaySize.x, 88});
            ImGui::Begin("ask", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
            ImGui::TextUnformatted("Ask kiln");
            ImGui::SetNextItemWidth(-64);
            bool submit = ImGui::InputText("##cmd", prompt_buf, sizeof(prompt_buf), ImGuiInputTextFlags_EnterReturnsTrue);
            ImGui::SameLine();
            if (ImGui::Button("Run")) submit = true;
            if (submit && prompt_buf[0]) {
                std::string reply;
                bool ok = handle_line(session, prompt_buf, reply);
                log.push_back(std::string("> ") + prompt_buf);
                log.push_back((ok ? "" : "error: ") + reply);
                prompt_buf[0] = 0;
                scene_dirty = true;
                ImGui::SetKeyboardFocusHere(-1);
            }
            ImGui::End();

            if (!io.WantCaptureMouse && glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
                double mx, my;
                glfwGetCursorPos(win, &mx, &my);
                float dx = float(mx - last_mx), dy = float(my - last_my);
                int ci = ensure_default_camera(session.doc.scene);
                session.doc.scene.nodes[ci].rotation_deg.y += dx * 0.35f;
                session.doc.scene.nodes[ci].rotation_deg.x -= dy * 0.35f;
            }
            glfwGetCursorPos(win, &last_mx, &last_my);

            ImGui::Render();

            vkWaitForFences(vk.device, 1, &vk.fences[vk.frame], VK_TRUE, UINT64_MAX);
            uint32_t img = 0;
            VkResult acq = vkAcquireNextImageKHR(vk.device, vk.swapchain, UINT64_MAX, vk.img_avail[vk.frame],
                                                 VK_NULL_HANDLE, &img);
            if (acq == VK_ERROR_OUT_OF_DATE_KHR) {
                recapture_swapchain(vk);
                continue;
            }
            VK_CHECK(vkResetFences(vk.device, 1, &vk.fences[vk.frame]));
            vkResetCommandBuffer(vk.cmds[vk.frame], 0);
            VkCommandBuffer cmd = vk.cmds[vk.frame];
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            VK_CHECK(vkBeginCommandBuffer(cmd, &bi));

            int ci = ensure_default_camera(session.doc.scene);
            const Node& cam = session.doc.scene.nodes[ci];
            Vec3 fwd = camera_forward(cam.rotation_deg);
            Mat4 view = look_at(cam.translation, cam.translation + fwd, {0, 1, 0});
            float aspect = vk.extent.height ? float(vk.extent.width) / float(vk.extent.height) : 1.0f;
            Mat4 proj = perspective(cam.fov_deg, aspect, 0.05f, 200.0f);
            FrameUBO ubo{};
            copy_mat(ubo.viewproj, mul(proj, view));
            Vec3 sun{0.45f, 0.82f, 0.35f};
            for (const auto& n : session.doc.scene.nodes) {
                if (n.type == NodeType::Light && n.light == LightType::Sun) {
                    sun = normalize(transform_dir(euler_xyz(n.rotation_deg), {0, 0, -1}));
                }
            }
            ubo.sun[0] = sun.x;
            ubo.sun[1] = sun.y;
            ubo.sun[2] = sun.z;
            ubo.sun[3] = 1;
            std::memcpy(vk.ubo_map, &ubo, sizeof(ubo));

            VkClearValue clears[2];
            clears[0].color = {{0.086f, 0.082f, 0.078f, 1}};
            clears[1].depthStencil = {1.0f, 0};
            VkRenderPassBeginInfo rpbi{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
            rpbi.renderPass = vk.rp;
            rpbi.framebuffer = vk.fbs[img];
            rpbi.renderArea.extent = vk.extent;
            rpbi.clearValueCount = 2;
            rpbi.pClearValues = clears;
            vkCmdBeginRenderPass(cmd, &rpbi, VK_SUBPASS_CONTENTS_INLINE);
            VkViewport viewport{0, 0, float(vk.extent.width), float(vk.extent.height), 0, 1};
            VkRect2D scissor{{0, 0}, vk.extent};
            vkCmdSetViewport(cmd, 0, 1, &viewport);
            vkCmdSetScissor(cmd, 0, 1, &scissor);
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, vk.pipeline);
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, vk.pl, 0, 1, &vk.dset, 0, nullptr);
            if (vk.vbo) {
                VkDeviceSize off = 0;
                vkCmdBindVertexBuffers(cmd, 0, 1, &vk.vbo, &off);
                for (const auto& d : vk.draws) {
                    PushPC pc{};
                    copy_mat(pc.model, d.model);
                    pc.color[0] = d.color.x;
                    pc.color[1] = d.color.y;
                    pc.color[2] = d.color.z;
                    pc.color[3] = 1;
                    vkCmdPushConstants(cmd, vk.pl, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(pc), &pc);
                    vkCmdDraw(cmd, d.count, 1, d.first, 0);
                }
            }
            ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), cmd);
            vkCmdEndRenderPass(cmd);
            VK_CHECK(vkEndCommandBuffer(cmd));

            VkPipelineStageFlags wait = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
            VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
            si.waitSemaphoreCount = 1;
            si.pWaitSemaphores = &vk.img_avail[vk.frame];
            si.pWaitDstStageMask = &wait;
            si.commandBufferCount = 1;
            si.pCommandBuffers = &cmd;
            si.signalSemaphoreCount = 1;
            si.pSignalSemaphores = &vk.render_done[vk.frame];
            VK_CHECK(vkQueueSubmit(vk.queue, 1, &si, vk.fences[vk.frame]));
            VkPresentInfoKHR pi{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
            pi.waitSemaphoreCount = 1;
            pi.pWaitSemaphores = &vk.render_done[vk.frame];
            pi.swapchainCount = 1;
            pi.pSwapchains = &vk.swapchain;
            pi.pImageIndices = &img;
            VkResult pres = vkQueuePresentKHR(vk.queue, &pi);
            if (pres == VK_ERROR_OUT_OF_DATE_KHR || pres == VK_SUBOPTIMAL_KHR) recapture_swapchain(vk);
            vk.frame = (vk.frame + 1) % VK::kMaxFrames;
        }

        vkDeviceWaitIdle(vk.device);
        ImGui_ImplVulkan_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    } catch (const std::exception& e) {
        std::cerr << "kiln gui: " << e.what() << "\n";
        std::cerr << "falling back to REPL. Build shaders with scripts/compile_shaders.sh\n";
        glfwDestroyWindow(win);
        glfwTerminate();
        return run_repl(session);
    }

    vkDeviceWaitIdle(vk.device);
    destroy_swapchain_objects(vk);
    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}

}  // namespace kiln
