group "default" {
  targets = ["base", "vnc"]
}

variable "COMMIT_SHA" {}

variable "GITHUB_REPO" {}

target "base" {
  context    = "."
  dockerfile = "Dockerfile.base"
  tags = [
    // If GITHUB_REPO is not set, then we are building locally
    equal("", GITHUB_REPO) ? "mep3:latest" : "",
    equal("", GITHUB_REPO) && notequal("", COMMIT_SHA) ? "mep3:${COMMIT_SHA}" : "",
    notequal("", GITHUB_REPO) ? "ghcr.io/${GITHUB_REPO}/mep3:latest" : "",
    notequal("", GITHUB_REPO) && notequal("", COMMIT_SHA) ? "ghcr.io/${GITHUB_REPO}:${COMMIT_SHA}" : ""
  ]
  // Enable layer caching
  cache-from = ["type=gha,scope=mep3"]
  cache-to   = ["type=gha,mode=max,scope=mep3"]
  // for reference, can be passed the same way as --build-arg, this can be completely omitted, and in this configuration will default to values specified in Dockerfile
  args = {
    DEBIAN_FRONTEND = null,
    UID = null,
  }
}

target "vnc" {
  context    = "."
  dockerfile = "Dockerfile.vnc"
  contexts = {
    mep3 = "target:base"
  }
  tags = [
    // If GITHUB_REPO is not set, then we are building locally
    equal("", GITHUB_REPO) ? "mep3-vnc:latest" : "",
    equal("", GITHUB_REPO) && notequal("", COMMIT_SHA) ? "mep3-vnc:${COMMIT_SHA}" : "",
    notequal("", GITHUB_REPO) ? "ghcr.io/${GITHUB_REPO}/mep3-vnc:latest" : "",
    notequal("", GITHUB_REPO) && notequal("", COMMIT_SHA) ? "ghcr.io/${GITHUB_REPO}-vnc:${COMMIT_SHA}" : ""
  ]
  // Enable layer caching
  cache-from = ["type=gha,scope=mep3-vnc"]
  cache-to   = ["type=gha,mode=max,scope=mep3-vnc"]
}
