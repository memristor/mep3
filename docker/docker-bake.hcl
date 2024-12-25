variable "COMMIT_SHA" {}

variable "GITHUB_REPO" {}

variable "TARGET_IMAGE_NAME_MAPPING" {
  default = {
    "base"   = "mep3"
    "vnc"    = "mep3-vnc"
    "deploy" = "mep3-deploy"
  }
}

variable CONTEXTS_MAPPING {
  default = {
    "vnc" = {
      "mep3" = "target:mep3"
    }
  }
}

function "eval_tags" {
  params = [image_name, commit_sha, github_repo]
  result = [
    // If GITHUB_REPO is not set, then we are building locally
    equal("", github_repo) ? image_name : "",
    equal("", github_repo) && notequal("", commit_sha) ? "${image_name}:${commit_sha}" : "",
    // otherwise, we are building on GitHub Actions
    notequal("", github_repo) ? "ghcr.io/memristor/${image_name}:latest" : "",
    notequal("", github_repo) && notequal("", commit_sha) ? "ghcr.io/memristor/${image_name}:${commit_sha}" : ""
  ]
}

target "default" {
  name = lookup(TARGET_IMAGE_NAME_MAPPING, tgt, "")
  matrix = {
    tgt = keys(TARGET_IMAGE_NAME_MAPPING)
  }

  tags = eval_tags(lookup(TARGET_IMAGE_NAME_MAPPING, tgt, ""), COMMIT_SHA, GITHUB_REPO)

  # Use Dockerfile.${tgt} for each target as defined by targets in TARGET_IMAGE_NAME_MAPPING
  dockerfile = "Dockerfile.${tgt}"

  # Cache settings, enable caching from previous builds
  cache-to   = [format("%s%s", "type=gha,mode=max,scope=", lookup(TARGET_IMAGE_NAME_MAPPING, tgt, ""))]
  cache-from = [format("%s%s", "type=gha,scope=", lookup(TARGET_IMAGE_NAME_MAPPING, tgt, ""))]

  # Contexts for dependent images, as defined in CONTEXTS_MAPPING graph, i.e. mep3-vnc builds atop mep3
  contexts = lookup(CONTEXTS_MAPPING, tgt, {})
}
